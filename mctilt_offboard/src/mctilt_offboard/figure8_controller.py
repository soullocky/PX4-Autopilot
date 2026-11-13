#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1/MAVROS figure-eight trajectory for PX4 Offboard flight tests.

Recommended real-flight workflow:
  1. Take off manually in Position mode and establish a stable hover.
  2. Start this node. It captures the current local pose as the trajectory center.
  3. The node pre-streams a hold setpoint and waits for the pilot to switch
     to OFFBOARD using the RC mode switch. It never changes flight mode.
  4. The vehicle follows a smooth Gerono figure eight, returns to the center,
     and keeps publishing a hold setpoint until the pilot changes mode.

The node does NOT arm or disarm the vehicle or change flight modes.
MCTilt pitch tracking requires the matching firmware and MC_MCT_OBP_EN=1.
The mask-71 attitude stream carries pitch only; position supplies yaw, and
PX4 calculates roll for lateral position control. Never use this stream with
stock firmware. control_pitch=false retains the original position-only node.

MAVROS ROS topics use ENU coordinates even though PositionTarget retains the
MAVLink enum name FRAME_LOCAL_NED. MAVROS performs the ENU-to-NED conversion.
"""

from __future__ import division, print_function

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from mavros_msgs.srv import ParamGet
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class FigureEightOffboard(object):
    """Generate and supervise a smooth local-frame figure-eight experiment."""

    def __init__(self):
        # Gentle defaults for the first real-flight test. Increase only after
        # checking tracking error, actuator saturation, and motion-capture delay.
        self.rate_hz = float(rospy.get_param('~rate', 50.0))
        # 八字轨迹的长、宽在这里设置，单位均为米（相对于启动时的悬停中心）：
        # x(t)=amplitude_x*sin(theta)，所以总长度=2*amplitude_x；默认 2.0 m。
        # y(t)=0.5*amplitude_y*sin(2*theta)，所以总宽度=amplitude_y；默认 0.6 m。
        # 若 align_with_initial_yaw=True，长轴会随起始偏航角旋转，而非固定沿 ENU 的 x 轴。
        self.amplitude_x = float(rospy.get_param('~amplitude_x', 1.0))
        self.amplitude_y = float(rospy.get_param('~amplitude_y', 0.60))
        self.period = float(rospy.get_param('~period', 18.0))
        self.cycles = int(rospy.get_param('~cycles', 3))
        self.ramp_time = float(rospy.get_param('~ramp_time', 4.0))
        self.prestream_time = float(rospy.get_param('~prestream_time', 3.0))
        self.settle_time = float(rospy.get_param('~settle_time', 10.0))
        self.final_hold_time = float(rospy.get_param('~final_hold_time', 5.0))
        self.pose_timeout = float(rospy.get_param('~pose_timeout', 0.5))
        self.state_timeout = float(rospy.get_param('~state_timeout', 3.0))
        self.max_tracking_error = float(rospy.get_param('~max_tracking_error', 1.0))
        self.error_timeout = float(rospy.get_param('~error_timeout', 0.6))
        self.require_armed = bool(rospy.get_param('~require_armed', True))
        self.use_velocity_ff = bool(rospy.get_param('~use_velocity_ff', True))
        self.use_acceleration_ff = bool(rospy.get_param('~use_acceleration_ff', False))
        self.align_with_initial_yaw = bool(rospy.get_param('~align_with_initial_yaw', True))

        # 仅 MCTilt：pitch_deg 是 NED/FRD 机体俯仰角，正值抬头，不是舵机角度。
        self.control_pitch = bool(rospy.get_param('~control_pitch', True))
        self.pitch_deg = float(rospy.get_param('~pitch_deg', 10.0))
        self.pitch_amplitude_deg = float(rospy.get_param('~pitch_amplitude_deg', 0.0))
        self.pitch_period = float(rospy.get_param('~pitch_period', self.period))
        self.pitch_ramp_time = float(rospy.get_param('~pitch_ramp_time', 8.0))
        self.max_pitch_error_deg = float(rospy.get_param('~max_pitch_error_deg', 15.0))
        self.pitch_error_timeout = float(rospy.get_param('~pitch_error_timeout', 1.0))
        self._initial_pitch = 0.0
        self._pitch_epoch = None
        self._trajectory_start = None
        self._return_start = None
        self._return_pitch = 0.0
        self._last_pitch = 0.0
        self._pitch_error_start = None

        numbers = (self.rate_hz, self.period, self.ramp_time, self.settle_time,
                   self.amplitude_x, self.amplitude_y, self.pitch_deg,
                   self.pitch_amplitude_deg, self.pitch_period, self.pitch_ramp_time,
                   self.max_pitch_error_deg, self.pitch_error_timeout, self.pose_timeout,
                   self.state_timeout, self.max_tracking_error, self.error_timeout,
                   self.prestream_time, self.final_hold_time)
        if not all(math.isfinite(x) for x in numbers):
            raise ValueError('Trajectory and pitch parameters must be finite')
        if self.control_pitch and (self.pitch_period <= 0.0 or self.pitch_ramp_time <= 0.0 or
                                   self.settle_time < self.pitch_ramp_time or
                                   self.max_pitch_error_deg <= 0.0 or self.pitch_error_timeout <= 0.0):
            raise ValueError('Pitch periods/limits must be positive; settle_time >= pitch_ramp_time')

        if self.rate_hz < 10.0:
            raise ValueError('~rate must be at least 10 Hz for robust Offboard streaming')
        if self.period <= 2.0 * self.ramp_time:
            raise ValueError('~period must be greater than 2*~ramp_time')
        if self.cycles < 1:
            raise ValueError('~cycles must be at least 1')
        if self.amplitude_x <= 0.0 or self.amplitude_y <= 0.0:
            raise ValueError('Figure-eight amplitudes must be positive')
        if self.ramp_time <= 0.0 or self.prestream_time < 1.0:
            raise ValueError('~ramp_time must be positive and ~prestream_time at least 1 s')
        if (self.pose_timeout <= 0.0 or self.state_timeout <= 0.0 or
                self.max_tracking_error <= 0.0 or self.error_timeout <= 0.0):
            raise ValueError('Pose, state and tracking-error limits must be positive')

        self._lock = threading.Lock()
        self._state = State()
        self._state_receive_time = rospy.Time(0)
        self._pose = None
        self._pose_receive_time = rospy.Time(0)

        self._center = None
        self._yaw_hold = 0.0
        self._path_rotation = 0.0
        self._last_reference = None
        self._error_start = None
        self._abort_reason = None

        self._setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self._pitch_pub = (rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
                           if self.control_pitch else None)
        self._reference_pub = rospy.Publisher(
            'mctilt_offboard/reference', PoseStamped, queue_size=20)
        self._phase_pub = rospy.Publisher(
            'mctilt_offboard/phase', String, queue_size=10, latch=True)

        rospy.Subscriber('mavros/state', State, self._state_callback, queue_size=10)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                         self._pose_callback, queue_size=20)

    def _state_callback(self, msg):
        with self._lock:
            self._state = msg
            self._state_receive_time = rospy.Time.now()

    def _pose_callback(self, msg):
        with self._lock:
            self._pose = msg
            self._pose_receive_time = rospy.Time.now()

    def _snapshot(self):
        with self._lock:
            return self._state, self._state_receive_time, self._pose, self._pose_receive_time

    def _state_is_fresh(self, state, state_time):
        return state.connected and (rospy.Time.now() - state_time).to_sec() < self.state_timeout

    @staticmethod
    def _pose_is_valid(pose):
        if pose is None:
            return False
        p, q = pose.pose.position, pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return (all(math.isfinite(v) for v in values) and
                0.81 < q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1.21)

    @staticmethod
    def _smoothstep5(u):
        """Return smootherstep and its first two derivatives versus u."""
        u = min(max(u, 0.0), 1.0)
        value = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
        first = 30.0 * u ** 2 * (1.0 - u) ** 2
        second = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u)
        return value, first, second

    def _envelope(self, t, duration):
        """Smoothly grow and remove trajectory amplitude at both ends."""
        if t < self.ramp_time:
            s, ds, dds = self._smoothstep5(t / self.ramp_time)
            return s, ds / self.ramp_time, dds / (self.ramp_time ** 2)

        if t > duration - self.ramp_time:
            u = (duration - t) / self.ramp_time
            s, ds, dds = self._smoothstep5(u)
            return s, -ds / self.ramp_time, dds / (self.ramp_time ** 2)

        return 1.0, 0.0, 0.0

    def _trajectory(self, t, duration):
        """Return ENU position, velocity, and acceleration at time t."""
        omega = 2.0 * math.pi / self.period
        theta = omega * t

        # Gerono 八字：x=A*sin(theta)，y=(B/2)*sin(2*theta)。
        # A 是长轴半幅、B 是短轴总宽；稳态轨迹的包络尺寸为 2A × B。
        # 起止阶段的平滑包络 env 会暂时缩小轨迹，不改变设定的最大长宽。
        base_x = self.amplitude_x * math.sin(theta)
        base_y = 0.5 * self.amplitude_y * math.sin(2.0 * theta)
        base_vx = self.amplitude_x * omega * math.cos(theta)
        base_vy = self.amplitude_y * omega * math.cos(2.0 * theta)
        base_ax = -self.amplitude_x * omega ** 2 * math.sin(theta)
        base_ay = -2.0 * self.amplitude_y * omega ** 2 * math.sin(2.0 * theta)

        env, env_dot, env_ddot = self._envelope(t, duration)
        x = env * base_x
        y = env * base_y
        vx = env_dot * base_x + env * base_vx
        vy = env_dot * base_y + env * base_vy
        ax = env_ddot * base_x + 2.0 * env_dot * base_vx + env * base_ax
        ay = env_ddot * base_y + 2.0 * env_dot * base_vy + env * base_ay

        # Optionally align the long axis of the path with the initial heading.
        c = math.cos(self._path_rotation)
        s = math.sin(self._path_rotation)
        xr, yr = c * x - s * y, s * x + c * y
        vxr, vyr = c * vx - s * vy, s * vx + c * vy
        axr, ayr = c * ax - s * ay, s * ax + c * ay

        position = (self._center[0] + xr, self._center[1] + yr, self._center[2])
        velocity = (vxr, vyr, 0.0)
        acceleration = (axr, ayr, 0.0)
        return position, velocity, acceleration

    def _hold_reference(self, position=None):
        if position is None:
            position = self._center
        return position, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    def _build_setpoint(self, position, velocity, acceleration):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        # The enum is the MAVLink target frame. The values placed in this ROS
        # message remain ENU; MAVROS converts them before sending to PX4.
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        mask = PositionTarget.IGNORE_YAW_RATE
        if not self.use_velocity_ff:
            mask |= PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
        if not self.use_acceleration_ff:
            mask |= (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                     PositionTarget.IGNORE_AFZ)
        msg.type_mask = mask

        msg.position.x, msg.position.y, msg.position.z = position
        msg.velocity.x, msg.velocity.y, msg.velocity.z = velocity
        (msg.acceleration_or_force.x,
         msg.acceleration_or_force.y,
         msg.acceleration_or_force.z) = acceleration
        msg.yaw = self._yaw_hold
        msg.yaw_rate = 0.0
        return msg

    def _publish_reference(self, position, velocity, acceleration):
        # 两条流使用同一规划时刻；它们不是原子消息，飞控各自检查有效期。
        now = rospy.Time.now()
        pitch = self._planned_pitch(now.to_sec()) if self.control_pitch else 0.0
        msg = self._build_setpoint(position, velocity, acceleration)
        msg.header.stamp = now
        q = quaternion_from_euler(0.0, -pitch, self._yaw_hold)
        if self.control_pitch:
            attitude = AttitudeTarget()
            attitude.header = msg.header
            # 71：忽略所有机体角速度和油门；只有专用固件可以与位置流同时使用。
            attitude.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE |
                                  AttitudeTarget.IGNORE_YAW_RATE | AttitudeTarget.IGNORE_THRUST)
            (attitude.orientation.x, attitude.orientation.y,
             attitude.orientation.z, attitude.orientation.w) = q
            attitude.thrust = 0.0
            self._pitch_pub.publish(attitude)
            self._last_pitch = pitch
        self._setpoint_pub.publish(msg)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.position
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self._reference_pub.publish(pose)
        self._last_reference = position

    def _planned_pitch(self, now):
        """与位置轨迹同钟规划：进入倾角、可选正弦变化、结束后平滑回正。"""
        if self._return_start is not None:
            blend = self._smoothstep5((now - self._return_start) / self.pitch_ramp_time)[0]
            return self._return_pitch * (1.0 - blend)
        if self._pitch_epoch is None:
            return self._initial_pitch
        blend = self._smoothstep5((now - self._pitch_epoch) / self.pitch_ramp_time)[0]
        pitch = self._initial_pitch + blend * (math.radians(self.pitch_deg) - self._initial_pitch)
        if self._trajectory_start is not None:
            t = max(0.0, now - self._trajectory_start)
            duration = self.cycles * self.period
            if t < duration:
                env = self._envelope(t, duration)[0]
                pitch += math.radians(self.pitch_amplitude_deg) * env * math.sin(2.0 * math.pi * t / self.pitch_period)
        return pitch

    def _start_pitch_return(self):
        """正常完成或中止时从当前发送值回正，避免目标突然跳变。"""
        self._return_pitch = self._last_pitch
        self._return_start = rospy.Time.now().to_sec()

    def _check_pitch_firmware(self):
        """先核对专用固件参数，失败时禁止发送姿态流，避免原版飞控误入姿态控制。"""
        if not self.control_pitch:
            return
        rospy.wait_for_service('mavros/param/get', timeout=10.0)
        get_param = rospy.ServiceProxy('mavros/param/get', ParamGet)
        def read(name, integer=False):
            result = get_param(param_id=name)
            if not result.success:
                raise RuntimeError('Missing FCU parameter: ' + name)
            return result.value.integer if integer else result.value.real
        for name, expected in [('CA_AIRFRAME', 8), ('CA_ROTOR_COUNT', 4),
                               ('CA_SV_TL_COUNT', 4), ('MC_MCT_OBP_EN', 1)]:
            if read(name, True) != expected:
                raise RuntimeError('MCTilt pitch requires %s=%s' % (name, expected))
        limit = read('MC_MCT_OBP_MAX')
        rate = read('MC_MCT_OBP_RATE')
        timeout = read('MC_MCT_OBP_TMO')
        if not all(math.isfinite(x) and x > 0 for x in (limit, rate, timeout)):
            raise RuntimeError('Invalid FCU pitch limits')
        if abs(self.pitch_deg) + abs(self.pitch_amplitude_deg) > limit:
            raise ValueError('Planned pitch exceeds MC_MCT_OBP_MAX')
        if abs(math.degrees(self._initial_pitch)) > limit:
            raise ValueError('Initial pitch exceeds MC_MCT_OBP_MAX; return to a verified small tilt before starting')
        # 五次过渡峰值导数为 1.875；提前拒绝明显快于飞控限速的规划。
        excursion = max(abs(self.pitch_deg - math.degrees(self._initial_pitch)),
                        abs(self.pitch_deg) + abs(self.pitch_amplitude_deg))
        slew = max(1.875 * excursion / self.pitch_ramp_time,
                   abs(self.pitch_amplitude_deg) * (2.0 * math.pi / self.pitch_period + 1.875 / self.ramp_time))
        if slew > rate or timeout < 3.0 / self.rate_hz:
            raise ValueError('Increase pitch_ramp_time/pitch_period or check FCU pitch rate/timeout')
        if float(rospy.get_param('mavros/setpoint_raw/thrust_scaling', -1.0)) <= 0.0:
            raise RuntimeError('Configure MAVROS setpoint_raw/thrust_scaling=1.0 before starting MAVROS')
        rospy.loginfo('MCTilt pitch interface verified: limit %.1f deg, rate %.1f deg/s', limit, rate)

    def _publish_phase(self, phase):
        self._phase_pub.publish(String(data=phase))
        rospy.loginfo('MCTilt phase: %s', phase)

    def _wait_for_fcu_and_pose(self):
        self._publish_phase('WAIT_FCU')
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            state, state_time, pose, pose_time = self._snapshot()
            pose_fresh = self._pose_is_valid(pose) and (rospy.Time.now() - pose_time).to_sec() < self.pose_timeout
            if self._state_is_fresh(state, state_time) and pose_fresh:
                return True
            rospy.logwarn_throttle(2.0, 'Waiting for FCU connection and fresh local pose...')
            rate.sleep()
        return False

    def _capture_center(self):
        _, _, pose, _ = self._snapshot()
        if not self._pose_is_valid(pose):
            raise RuntimeError('Cannot capture invalid local pose')
        p = pose.pose.position
        q = pose.pose.orientation
        self._center = (p.x, p.y, p.z)
        self._yaw_hold = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        # ROS ENU/FLU 的欧拉 Pitch 与本接口 NED/FRD Pitch 符号相反。
        self._initial_pitch = -euler_from_quaternion((q.x, q.y, q.z, q.w))[1]
        self._path_rotation = self._yaw_hold if self.align_with_initial_yaw else 0.0
        rospy.loginfo('Captured ENU center: [%.3f, %.3f, %.3f], yaw %.1f deg',
                      self._center[0], self._center[1], self._center[2],
                      math.degrees(self._yaw_hold))

    def _stream_hold_for(self, seconds, phase):
        self._publish_phase(phase)
        rate = rospy.Rate(self.rate_hz)
        end = rospy.Time.now() + rospy.Duration(seconds)
        hold = self._hold_reference()
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if phase != 'PRESTREAM' and not self._tracking_is_safe(hold[0]):
                return False
            state, state_time, _, pose_time = self._snapshot()
            if (not self._state_is_fresh(state, state_time) or
                    (rospy.Time.now() - pose_time).to_sec() > self.pose_timeout or
                    (self.require_armed and not state.armed)):
                self._abort_reason = 'state/pose timeout during hold'
                return False
            self._publish_reference(*hold)
            rate.sleep()
        return not rospy.is_shutdown()

    def _wait_for_pilot_offboard(self):
        """Continuously stream a hold target until the RC switch enters OFFBOARD."""
        self._publish_phase('WAIT_OFFBOARD')
        rospy.loginfo('Hold setpoints are streaming. Switch to OFFBOARD with the RC when ready.')
        rate = rospy.Rate(self.rate_hz)
        hold = self._hold_reference()

        while not rospy.is_shutdown():
            self._publish_reference(*hold)
            state, state_time, _, pose_time = self._snapshot()
            if not self._state_is_fresh(state, state_time):
                rospy.logerr('FCU state lost while waiting for OFFBOARD: connected=%s, age=%.2f s (limit %.2f s).',
                             state.connected, (rospy.Time.now() - state_time).to_sec(), self.state_timeout)
                return False
            if (rospy.Time.now() - pose_time).to_sec() > self.pose_timeout:
                rospy.logerr('Local pose lost while waiting for OFFBOARD: age=%.2f s (limit %.2f s).',
                             (rospy.Time.now() - pose_time).to_sec(), self.pose_timeout)
                return False
            if self.require_armed and not state.armed:
                rospy.logerr('Vehicle disarmed while waiting for OFFBOARD.')
                return False
            if state.mode == 'OFFBOARD':
                self._pitch_epoch = rospy.Time.now().to_sec()
                return True
            rate.sleep()
        return False

    def _tracking_is_safe(self, reference):
        state, state_time, pose, pose_time = self._snapshot()
        now = rospy.Time.now()

        if not self._state_is_fresh(state, state_time):
            self._abort_reason = 'FCU connection/state timeout'
            return False
        if not self._pose_is_valid(pose) or (now - pose_time).to_sec() > self.pose_timeout:
            self._abort_reason = 'local pose timeout'
            return False
        if state.mode != 'OFFBOARD':
            self._abort_reason = 'vehicle left OFFBOARD mode'
            return False
        if self.require_armed and not state.armed:
            self._abort_reason = 'vehicle is not armed'
            return False

        p = pose.pose.position
        error = math.sqrt((p.x - reference[0]) ** 2 +
                          (p.y - reference[1]) ** 2 +
                          (p.z - reference[2]) ** 2)
        if error > self.max_tracking_error:
            if self._error_start is None:
                self._error_start = now
            elif (now - self._error_start).to_sec() > self.error_timeout:
                self._abort_reason = 'tracking error %.2f m exceeded limit %.2f m' % (
                    error, self.max_tracking_error)
                return False
        else:
            self._error_start = None
        if self.control_pitch:
            q = pose.pose.orientation
            actual_pitch = -euler_from_quaternion((q.x, q.y, q.z, q.w))[1]
            pitch_error = abs(math.degrees(actual_pitch - self._last_pitch))
            if not math.isfinite(pitch_error) or pitch_error > self.max_pitch_error_deg:
                if self._pitch_error_start is None:
                    self._pitch_error_start = now
                elif (now - self._pitch_error_start).to_sec() > self.pitch_error_timeout:
                    self._abort_reason = 'pitch tracking error %.1f deg' % pitch_error
                    return False
            else:
                self._pitch_error_start = None
        return True

    def _run_trajectory(self):
        self._publish_phase('TRACKING')
        rate = rospy.Rate(self.rate_hz)
        duration = self.cycles * self.period
        start = rospy.Time.now()
        self._trajectory_start = start.to_sec()

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start).to_sec()
            if t >= duration:
                return True

            reference = self._trajectory(t, duration)
            self._publish_reference(*reference)
            if not self._tracking_is_safe(reference[0]):
                return False
            rate.sleep()
        return False

    def _abort_to_current_hold(self):
        self._start_pitch_return()
        _, _, pose, pose_time = self._snapshot()
        if pose is not None and (rospy.Time.now() - pose_time).to_sec() < self.pose_timeout:
            p = pose.pose.position
            self._center = (p.x, p.y, p.z)
        elif self._last_reference is not None:
            self._center = self._last_reference
        self._publish_phase('ABORT_HOLD')
        rospy.logerr('Trajectory aborted: %s. Holding current position; pilot should take over.',
                     self._abort_reason)

    def _hold_until_shutdown(self, initial_seconds=None):
        phase = 'FINAL_HOLD' if self._abort_reason is None else 'ABORT_HOLD'
        self._publish_phase(phase)
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            state, state_time, _, pose_time = self._snapshot()
            if state.mode != 'OFFBOARD':
                rospy.loginfo('Pilot left OFFBOARD; MCTilt test node is stopping.')
                return
            # 定位/连接丢失时停止两条流，让 PX4 原有 Offboard 失联保护接管。
            if (not self._state_is_fresh(state, state_time) or
                    (rospy.Time.now() - pose_time).to_sec() > self.pose_timeout or
                    (self.require_armed and not state.armed)):
                rospy.logerr('Hold stream stopped: state/pose/arming invalid; pilot must take over.')
                return
            self._publish_reference(*self._hold_reference())
            if initial_seconds is not None:
                elapsed = (rospy.Time.now() - start).to_sec()
                if elapsed >= initial_seconds:
                    rospy.loginfo_throttle(5.0,
                                           'Experiment complete; switch out of OFFBOARD and land manually.')
            rate.sleep()

    def run(self):
        if not self._wait_for_fcu_and_pose():
            return
        self._capture_center()
        self._check_pitch_firmware()

        state, _, _, _ = self._snapshot()
        if self.require_armed and not state.armed:
            rospy.logerr('Vehicle must already be armed and stably hovering before this test.')
            return

        if state.mode == 'OFFBOARD':
            raise RuntimeError('Start in Position mode; use the RC switch after prestreaming')
        if not self._stream_hold_for(self.prestream_time, 'PRESTREAM'):
            return
        if not self._wait_for_pilot_offboard():
            return

        if not self._stream_hold_for(self.settle_time, 'SETTLE'):
            self._abort_to_current_hold()
            self._hold_until_shutdown()
            return
        state, state_time, _, pose_time = self._snapshot()
        if (not self._state_is_fresh(state, state_time) or state.mode != 'OFFBOARD' or (self.require_armed and not state.armed)
                or (rospy.Time.now() - pose_time).to_sec() > self.pose_timeout):
            rospy.logwarn('OFFBOARD, arming, or local pose lost during settling; trajectory not started.')
            return
        success = self._run_trajectory()
        if not success:
            self._abort_to_current_hold()
        else:
            self._start_pitch_return()
        self._hold_until_shutdown(self.final_hold_time)


def main():
    # 两个测试入口使用同一 ROS 节点名，避免误把它们作为独立控制源同时启动。
    rospy.init_node('mctilt_offboard', anonymous=False)
    try:
        node = FigureEightOffboard()
        node.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as exc:  # Keep a clear error in roslaunch/rosout.
        rospy.logfatal('Figure-eight node failed: %s', exc)
        raise


if __name__ == '__main__':
    main()
