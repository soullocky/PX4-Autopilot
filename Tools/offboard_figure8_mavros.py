#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS1/MAVROS figure-eight trajectory for PX4 Offboard flight tests.

Recommended real-flight workflow:
  1. Take off manually in Position mode and establish a stable hover.
  2. Start this node. It captures the current local pose as the trajectory center.
  3. The node pre-streams a hold setpoint and optionally requests OFFBOARD.
  4. The vehicle follows a smooth Gerono figure eight, returns to the center,
     and keeps publishing a hold setpoint until the pilot changes mode.

The node does NOT arm or disarm the vehicle. For the modified Tiltquad/MCTilt
position controller, fixed roll/pitch attitude is still commanded through
AUX5/AUX6. Do not publish an attitude setpoint at the same time as this node.

MAVROS ROS topics use ENU coordinates even though PositionTarget retains the
MAVLink enum name FRAME_LOCAL_NED. MAVROS performs the ENU-to-NED conversion.
"""

from __future__ import division, print_function

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class FigureEightOffboard(object):
    """Generate and supervise a smooth local-frame figure-eight experiment."""

    def __init__(self):
        # Gentle defaults for the first real-flight test. Increase only after
        # checking tracking error, actuator saturation, and motion-capture delay.
        self.rate_hz = float(rospy.get_param('~rate', 50.0))
        self.amplitude_x = float(rospy.get_param('~amplitude_x', 1.0))
        self.amplitude_y = float(rospy.get_param('~amplitude_y', 0.60))
        self.period = float(rospy.get_param('~period', 18.0))
        self.cycles = int(rospy.get_param('~cycles', 3))
        self.ramp_time = float(rospy.get_param('~ramp_time', 4.0))
        self.prestream_time = float(rospy.get_param('~prestream_time', 3.0))
        self.settle_time = float(rospy.get_param('~settle_time', 4.0))
        self.final_hold_time = float(rospy.get_param('~final_hold_time', 5.0))
        self.offboard_timeout = float(rospy.get_param('~offboard_timeout', 10.0))
        self.pose_timeout = float(rospy.get_param('~pose_timeout', 0.5))
        self.max_tracking_error = float(rospy.get_param('~max_tracking_error', 1.0))
        self.error_timeout = float(rospy.get_param('~error_timeout', 0.6))
        self.request_offboard = bool(rospy.get_param('~request_offboard', True))
        self.require_armed = bool(rospy.get_param('~require_armed', True))
        self.use_velocity_ff = bool(rospy.get_param('~use_velocity_ff', True))
        self.use_acceleration_ff = bool(rospy.get_param('~use_acceleration_ff', False))
        self.align_with_initial_yaw = bool(rospy.get_param('~align_with_initial_yaw', True))

        if self.rate_hz < 10.0:
            raise ValueError('~rate must be at least 10 Hz for robust Offboard streaming')
        if self.period <= 2.0 * self.ramp_time:
            raise ValueError('~period must be greater than 2*~ramp_time')
        if self.cycles < 1:
            raise ValueError('~cycles must be at least 1')
        if self.amplitude_x <= 0.0 or self.amplitude_y <= 0.0:
            raise ValueError('Figure-eight amplitudes must be positive')

        self._lock = threading.Lock()
        self._state = State()
        self._pose = None
        self._pose_receive_time = rospy.Time(0)

        self._center = None
        self._yaw_hold = 0.0
        self._path_rotation = 0.0
        self._last_reference = None
        self._error_start = None
        self._abort_reason = None

        self._setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/local', PositionTarget, queue_size=20)
        self._reference_pub = rospy.Publisher(
            'figure8/reference', PoseStamped, queue_size=20)
        self._phase_pub = rospy.Publisher(
            'figure8/phase', String, queue_size=10, latch=True)

        rospy.Subscriber('mavros/state', State, self._state_callback, queue_size=10)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                         self._pose_callback, queue_size=20)
        self._set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

    def _state_callback(self, msg):
        with self._lock:
            self._state = msg

    def _pose_callback(self, msg):
        with self._lock:
            self._pose = msg
            self._pose_receive_time = rospy.Time.now()

    def _snapshot(self):
        with self._lock:
            return self._state, self._pose, self._pose_receive_time

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

        # Gerono lemniscate: x=A*sin(theta), y=B*sin(theta)*cos(theta).
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
        msg = self._build_setpoint(position, velocity, acceleration)
        self._setpoint_pub.publish(msg)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.position
        q = quaternion_from_euler(0.0, 0.0, self._yaw_hold)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self._reference_pub.publish(pose)
        self._last_reference = position

    def _publish_phase(self, phase):
        self._phase_pub.publish(String(data=phase))
        rospy.loginfo('Figure-eight phase: %s', phase)

    def _wait_for_fcu_and_pose(self):
        self._publish_phase('WAIT_FCU')
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            state, pose, pose_time = self._snapshot()
            pose_fresh = pose is not None and (rospy.Time.now() - pose_time).to_sec() < self.pose_timeout
            if state.connected and pose_fresh:
                return True
            rospy.logwarn_throttle(2.0, 'Waiting for FCU connection and fresh local pose...')
            rate.sleep()
        return False

    def _capture_center(self):
        _, pose, _ = self._snapshot()
        p = pose.pose.position
        q = pose.pose.orientation
        self._center = (p.x, p.y, p.z)
        self._yaw_hold = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
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
            self._publish_reference(*hold)
            rate.sleep()

    def _enter_offboard(self):
        if not self.request_offboard:
            rospy.logwarn('Automatic OFFBOARD request disabled; waiting for pilot mode switch.')

        self._publish_phase('WAIT_OFFBOARD')
        rate = rospy.Rate(self.rate_hz)
        deadline = rospy.Time.now() + rospy.Duration(self.offboard_timeout)
        last_request = rospy.Time(0)
        hold = self._hold_reference()

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_reference(*hold)
            state, _, _ = self._snapshot()
            if state.mode == 'OFFBOARD':
                return True

            if self.request_offboard and (rospy.Time.now() - last_request).to_sec() > 1.0:
                try:
                    response = self._set_mode(base_mode=0, custom_mode='OFFBOARD')
                    if not response.mode_sent:
                        rospy.logwarn('FCU rejected OFFBOARD request')
                except rospy.ServiceException as exc:
                    rospy.logwarn('OFFBOARD service call failed: %s', exc)
                last_request = rospy.Time.now()
            rate.sleep()

        rospy.logerr('OFFBOARD was not entered within %.1f s', self.offboard_timeout)
        return False

    def _tracking_is_safe(self, reference):
        state, pose, pose_time = self._snapshot()
        now = rospy.Time.now()

        if (now - pose_time).to_sec() > self.pose_timeout:
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
        return True

    def _run_trajectory(self):
        self._publish_phase('TRACKING')
        rate = rospy.Rate(self.rate_hz)
        duration = self.cycles * self.period
        start = rospy.Time.now()

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
        _, pose, _ = self._snapshot()
        if pose is not None:
            p = pose.pose.position
            self._center = (p.x, p.y, p.z)
        self._publish_phase('ABORT_HOLD')
        rospy.logerr('Trajectory aborted: %s. Holding current position; pilot should take over.',
                     self._abort_reason)

    def _hold_until_shutdown(self, initial_seconds=None):
        phase = 'FINAL_HOLD' if self._abort_reason is None else 'ABORT_HOLD'
        self._publish_phase(phase)
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
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

        state, _, _ = self._snapshot()
        if self.require_armed and not state.armed:
            rospy.logerr('Vehicle must already be armed and stably hovering before this test.')
            return

        self._stream_hold_for(self.prestream_time, 'PRESTREAM')
        if not self._enter_offboard():
            return

        self._stream_hold_for(self.settle_time, 'SETTLE')
        success = self._run_trajectory()
        if not success:
            self._abort_to_current_hold()
        self._hold_until_shutdown(self.final_hold_time)


def main():
    rospy.init_node('offboard_figure8_mavros', anonymous=False)
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
