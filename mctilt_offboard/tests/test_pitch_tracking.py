"""无需 ROS master 的接口/规划回归测试；不代替 MAVROS/飞控联调或实飞。"""
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock


class Obj(types.SimpleNamespace):
    pass


class Stamp:
    current = 100.0

    def __init__(self, value=0):
        self.value = value

    def to_sec(self):
        return self.value

    def __sub__(self, other):
        return Stamp(self.value - other.value)

    @classmethod
    def now(cls):
        return cls(cls.current)


class Message:
    FRAME_LOCAL_NED = 1
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
    IGNORE_YAW_RATE = 2048

    def __init__(self):
        self.header = Obj(stamp=Stamp(), frame_id='')
        self.position = Obj(x=0., y=0., z=0.)
        self.velocity = Obj(x=0., y=0., z=0.)
        self.acceleration_or_force = Obj(x=0., y=0., z=0.)
        self.orientation = Obj(x=0., y=0., z=0., w=1.)
        self.body_rate = Obj(x=0., y=0., z=0.)
        self.pose = Obj(position=self.position, orientation=self.orientation)


class Attitude(Message):
    IGNORE_ROLL_RATE, IGNORE_PITCH_RATE, IGNORE_YAW_RATE, IGNORE_THRUST = 1, 2, 4, 64


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def euler(q):
    x, y, z, w = q
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def qmul(a, b):
    x, y, z, w = a; X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


params = {}
rospy = types.ModuleType('rospy')
rospy.Time = Stamp
rospy.get_param = lambda key, default=None: params.get(key, default)
for name in ['Publisher', 'Subscriber', 'loginfo', 'logwarn_throttle', 'wait_for_service']:
    setattr(rospy, name, Mock())
for name in ['geometry_msgs.msg', 'mavros_msgs.msg', 'mavros_msgs.srv', 'std_msgs.msg', 'tf.transformations']:
    sys.modules[name] = types.ModuleType(name)
sys.modules['rospy'] = rospy
sys.modules['geometry_msgs.msg'].PoseStamped = Message
sys.modules['mavros_msgs.msg'].PositionTarget = Message
sys.modules['mavros_msgs.msg'].AttitudeTarget = Attitude
sys.modules['mavros_msgs.msg'].State = lambda: Obj(connected=True, armed=True, mode='POSCTL')
sys.modules['mavros_msgs.srv'].ParamGet = object
sys.modules['std_msgs.msg'].String = Obj
sys.modules['tf.transformations'].euler_from_quaternion = euler
sys.modules['tf.transformations'].quaternion_from_euler = quaternion
sys.path.insert(0, str(Path(__file__).parents[1] / 'src'))
from mctilt_offboard import figure8_controller as script
from mctilt_offboard.pitch_hold import PitchHoldOffboard


class PitchTrackingTest(unittest.TestCase):
    def setUp(self):
        params.clear()
        rospy.Publisher.side_effect = lambda *a, **kw: Mock()
        Stamp.current = 100.

    def make(self):
        node = script.FigureEightOffboard()
        node._center = (1., 2., 3.)
        node._yaw_hold = 1.1
        return node

    def test_smooth_entry_and_return(self):
        n = self.make(); n._initial_pitch = math.radians(-5); n._pitch_epoch = 100.
        self.assertAlmostEqual(n._planned_pitch(100), math.radians(-5))
        self.assertAlmostEqual(n._planned_pitch(108), math.radians(10))
        n._last_pitch = n._planned_pitch(108); Stamp.current = 108; n._start_pitch_return()
        self.assertAlmostEqual(n._planned_pitch(108), math.radians(10))
        self.assertAlmostEqual(n._planned_pitch(116), 0.)

    def test_sinusoidal_reference_is_bounded_and_continuous(self):
        params['~pitch_amplitude_deg'] = 5.
        n = self.make(); n._pitch_epoch = 90.; n._trajectory_start = 100.
        self.assertAlmostEqual(n._planned_pitch(100), math.radians(10))
        for i in range(540):
            self.assertLessEqual(abs(n._planned_pitch(100+i*.1)-math.radians(10)), math.radians(5)+1e-9)
        self.assertAlmostEqual(n._planned_pitch(154), math.radians(10))

    def test_mask_stamp_and_enu_frd_conversion(self):
        n = self.make(); n._pitch_epoch = 0.
        for pitch in [-60, -10, 0, 10, 60]:
            n.pitch_deg = pitch
            for yaw in [-2., 0., 1.7]:
                n._yaw_hold = yaw
                n._publish_reference(*n._hold_reference())
                a = n._pitch_pub.publish.call_args[0][0]
                p = n._setpoint_pub.publish.call_args[0][0]
                self.assertEqual(a.type_mask, 71)
                self.assertEqual(a.header.stamp, p.header.stamp)
                self.assertEqual(a.thrust, 0.)
                q = (a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w)
                # MAVROS: NED<-ENU 旋转左乘，FLU<-FRD 旋转右乘。
                converted = qmul(qmul(quaternion(math.pi, 0., math.pi/2), q), quaternion(math.pi, 0., 0.))
                self.assertAlmostEqual(euler(converted)[1], math.radians(pitch), places=6)

    def test_position_only_has_no_attitude_publisher(self):
        params['~control_pitch'] = False
        n = self.make(); self.assertIsNone(n._pitch_pub)
        n._publish_reference(*n._hold_reference())
        n._setpoint_pub.publish.assert_called_once()

    def test_invalid_parameters_and_pose(self):
        params['~pitch_deg'] = float('nan')
        with self.assertRaises(ValueError): self.make()
        params.clear(); params['~settle_time'] = 1.
        with self.assertRaises(ValueError): self.make()
        p = Message(); self.assertTrue(script.FigureEightOffboard._pose_is_valid(p))
        p.pose.position.x = float('nan')
        self.assertFalse(script.FigureEightOffboard._pose_is_valid(p))

    def test_firmware_handshake_rejects_stock_and_limit_violation(self):
        n = self.make()
        fc = dict(CA_AIRFRAME=8, CA_ROTOR_COUNT=4, CA_SV_TL_COUNT=4, MC_MCT_OBP_EN=1,
                  MC_MCT_OBP_MAX=20., MC_MCT_OBP_RATE=5., MC_MCT_OBP_TMO=.5)
        rospy.ServiceProxy = lambda *a: lambda param_id: Obj(success=param_id in fc,
                            value=Obj(integer=fc.get(param_id, 0), real=fc.get(param_id, 0)))
        params['mavros/setpoint_raw/thrust_scaling'] = 1.
        n._check_pitch_firmware()
        fc['MC_MCT_OBP_EN'] = 0
        with self.assertRaises(RuntimeError): n._check_pitch_firmware()
        fc['MC_MCT_OBP_EN'] = 1; n.pitch_deg = 60
        with self.assertRaises(ValueError): n._check_pitch_firmware()
        fc['MC_MCT_OBP_MAX'] = 60.
        with self.assertRaises(ValueError): n._check_pitch_firmware()  # 8 s 的大角度过渡过快

    def test_persistent_pitch_error_aborts(self):
        n = self.make(); n._last_pitch = math.radians(30)
        pose = Message(); pose.position.x, pose.position.y, pose.position.z = n._center
        n._snapshot = lambda: (Obj(connected=True, armed=True, mode='OFFBOARD'), Stamp.now(), pose, Stamp.now())
        self.assertTrue(n._tracking_is_safe(n._center))
        Stamp.current += 1.1
        self.assertFalse(n._tracking_is_safe(n._center))
        self.assertIn('pitch tracking error', n._abort_reason)

    def test_pitch_hold_never_generates_position_motion(self):
        n = PitchHoldOffboard(); n._center = (1.5, -2., 0.8)
        n._yaw_hold = 0.7; n._pitch_epoch = 100.
        for t in [0., 3., 8., 20., 50.]:
            position, velocity, acceleration = n._trajectory(t, 50.)
            self.assertEqual(position, n._center)
            self.assertEqual(velocity, (0., 0., 0.))
            self.assertEqual(acceleration, (0., 0., 0.))
            self.assertAlmostEqual(n._yaw_hold, 0.7)
        self.assertAlmostEqual(n._planned_pitch(108), math.radians(10))
        self.assertAlmostEqual(n._planned_pitch(128), math.radians(10))

    def test_pitch_hold_uses_safety_checked_hold_and_propagates_abort(self):
        params['~pitch_hold_time'] = 25.
        n = PitchHoldOffboard()
        n._stream_hold_for = Mock(return_value=False)
        self.assertFalse(n._run_trajectory())
        n._stream_hold_for.assert_called_once_with(25., 'PITCH_HOLD')
        self.assertIsNone(n._trajectory_start)

    def test_pitch_hold_rejects_ambiguous_or_invalid_configuration(self):
        for key, value in [('~pitch_hold_time', 0.), ('~pitch_hold_time', float('nan')),
                           ('~control_pitch', False), ('~pitch_amplitude_deg', 2.)]:
            params.clear(); params[key] = value
            with self.assertRaises(ValueError): PitchHoldOffboard()


if __name__ == '__main__':
    unittest.main()
