import unittest
from tests.core import TestCore
from pyrep.objects.joint import Joint
from pyrep.const import JointType, JointMode


class TestJoints(TestCore):

    def setUp(self):
        super().setUp()
        self.prismatic = Joint('prismatic_joint')
        self.prismatic_ctr = Joint('prismatic_joint_control_loop')
        self.revolute = Joint('revolute_joint')

    def test_get_joint_type(self):
        self.assertEqual(self.prismatic.get_joint_type(), JointType.PRISMATIC)

    def test_get_set_joint_mode(self):
        self.prismatic.set_joint_mode(JointMode.IK)
        self.assertEqual(self.prismatic.get_joint_mode(), JointMode.IK)

    def test_get_set_joint_position(self):
        self.prismatic.set_joint_position(0.5)
        pos = self.prismatic.get_joint_position()
        self.assertEqual(pos, 0.5)

    def test_get_set_joint_target_position(self):
        self.prismatic_ctr.set_joint_target_position(0.5)
        pos = self.prismatic_ctr.get_joint_target_position()
        self.assertEqual(pos, 0.5)
        # Now step a few times to drive the joint
        [self.pyrep.step() for _ in range(10)]
        self.assertAlmostEqual(
            self.prismatic_ctr.get_joint_position(), 0.5, delta=0.01)

    def test_get_set_joint_target_velocity(self):
        self.prismatic.set_joint_target_velocity(5.0)
        vel = self.prismatic.get_joint_target_velocity()
        self.assertEqual(vel, 5.0)
        # Now step a few times to drive the joint
        [self.pyrep.step() for _ in range(10)]
        self.assertAlmostEqual(
            self.prismatic.get_joint_position(), 0.5, delta=0.01)

    def test_get_set_joint_force(self):
        [self.pyrep.step() for _ in range(10)]
        # Set a really high velocity (torque control)
        self.prismatic.set_joint_target_velocity(-99999)
        self.prismatic.set_joint_force(0.6)
        self.pyrep.step()
        force = self.prismatic.get_joint_force()
        self.assertAlmostEqual(force, 0.6, delta=0.01)

    def test_get_velocity(self):
        self.prismatic.set_joint_target_velocity(0.5)
        self.pyrep.step()
        self.assertGreater(self.prismatic.get_joint_velocity(), 0.1)

    def test_get_set_joint_interval(self):
        self.revolute.set_joint_interval(False, [-2.0, 2.0])
        cyclic, interval = self.revolute.get_joint_interval()
        self.assertFalse(cyclic)
        self.assertEqual(interval, [-2.0, 2.0])

    def test_get_joint_upper_velocity_limit(self):
        limit = self.prismatic.get_joint_upper_velocity_limit()
        self.assertEqual(limit, 10.0)

    def test_get_set_control_loop_enabled(self):
        self.assertFalse(self.prismatic.is_control_loop_enabled())
        self.prismatic.set_control_loop_enabled(True)
        self.assertTrue(self.prismatic.is_control_loop_enabled())

    def test_get_set_motor_enabled(self):
        self.assertTrue(self.prismatic.is_motor_enabled())
        self.prismatic.set_motor_enabled(False)
        self.assertFalse(self.prismatic.is_motor_enabled())

    def test_get_set_motor_locked_at_zero_velocity(self):
        self.assertFalse(self.prismatic.is_motor_locked_at_zero_velocity())
        self.prismatic.set_motor_locked_at_zero_velocity(True)
        self.assertTrue(self.prismatic.is_motor_locked_at_zero_velocity())


if __name__ == '__main__':
    unittest.main()
