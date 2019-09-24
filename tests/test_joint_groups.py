import unittest
from tests.core import TestCore
from pyrep.const import JointType
from pyrep.robots.arms.panda import Panda
import numpy as np


# Pick one arm to test all of the joint group functionality.
# Simply checks for wiring mistakes between the joints.
class TestJointGroups(TestCore):

    def setUp(self):
        super().setUp()
        self.robot = Panda()
        self.num_joints = len(self.robot.joints)

    def test_get_joint_types(self):
        self.assertEqual(self.robot.get_joint_types(),
                         [JointType.REVOLUTE] * self.num_joints)

    def test_get_set_joint_positions(self):
        self.robot.set_joint_positions([0.1] * self.num_joints)
        self.assertEqual(len(self.robot.get_joint_positions()), self.num_joints)

    def test_get_set_joint_target_positions(self):
        self.robot.set_joint_target_positions([0.1] * self.num_joints)
        self.assertEqual(
            len(self.robot.get_joint_target_positions()), self.num_joints)

    def test_get_set_joint_target_velocities(self):
        self.robot.set_joint_target_velocities([0.1] * self.num_joints)
        self.assertEqual(
            len(self.robot.get_joint_target_velocities()), self.num_joints)

    def test_get_set_joint_forces(self):
        self.robot.set_joint_target_velocities([-999] * self.num_joints)
        self.robot.set_joint_forces([0.6] * self.num_joints)
        self.pyrep.step()
        self.assertEqual(len(self.robot.get_joint_forces()), self.num_joints)

    def test_get_velocities(self):
        self.assertEqual(
            len(self.robot.get_joint_velocities()), self.num_joints)

    def test_get_set_joint_intervals(self):
        self.robot.set_joint_intervals(
            [False] * self.num_joints, [[-2.0, 2.0]] * self.num_joints)
        cyclics, intervals = self.robot.get_joint_intervals()
        self.assertEqual(len(cyclics), self.num_joints)
        self.assertEqual(len(intervals), self.num_joints)

    def test_get_joint_upper_velocity_limits(self):
        self.assertEqual(
            len(self.robot.get_joint_upper_velocity_limits()), self.num_joints)

    def test_get_visuals(self):
        self.assertEqual(len(self.robot.get_visuals()), 10)


if __name__ == '__main__':
    unittest.main()
