import unittest
from tests.core import TestCore
from pyrep import PyRep
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.objects.dummy import Dummy
import numpy as np
from os import path

from pyrep.robots.arms.panda import Panda
from pyrep.robots.arms.mico import Mico
from pyrep.robots.arms.jaco import Jaco
from pyrep.robots.arms.sawyer import Sawyer
from pyrep.robots.arms.baxter import BaxterLeft, BaxterRight
from pyrep.robots.arms.lbr_iiwa_7_r800 import LBRIwaa7R800
from pyrep.robots.arms.lbr_iiwa_14_r820 import LBRIwaa14R820
from pyrep.robots.arms.ur3 import UR3
from pyrep.robots.arms.ur5 import UR5
from pyrep.robots.arms.ur10 import UR10

ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets')


ARMS = [
    ('Panda', Panda),
    ('Sawyer', Sawyer),
    ('Mico', Mico),
    ('Jaco', Jaco),
    ('Baxter_leftArm', BaxterLeft),
    ('Baxter_rightArm', BaxterRight),
    ('LBR_iiwa_7_R800', LBRIwaa7R800),
    ('LBR_iiwa_14_R820', LBRIwaa14R820),
    ('UR3', UR3),
    ('UR5', UR5),
    ('UR10', UR10),
]


class TestArmsAndConfigurationPaths(TestCore):

    def setUp(self):
        self.pyrep = PyRep()
        self.pyrep.launch(path.join(
            ASSET_DIR, 'test_scene_robots.ttt'), headless=True)
        self.pyrep.step()
        self.pyrep.start()

    # It is enough to only test the constructor of each arm (in there we make
    # assumptions about the structure of the arm model). All other tests
    # can be run on one arm.
    def test_get_arm(self):
        for arm_name, arm_type in ARMS:
            with self.subTest(arm=arm_name):
                arm = arm_type()
                self.assertIsInstance(arm, arm_type)

    def test_get_configs_for_tip_pose(self):
        arm = Panda()
        waypoint = Dummy('Panda_waypoint')
        configs = arm.get_configs_for_tip_pose(
            waypoint.get_position(), waypoint.get_orientation())
        self.assertIsNotNone(configs)

    def test_get_path_from_cartesian_path(self):
        arm = Panda()
        cartesian_path = CartesianPath('Panda_cartesian_path')
        path = arm.get_path_from_cartesian_path(cartesian_path)
        self.assertIsNotNone(path)

    def test_get_linear_path(self):
        arm = Panda()
        waypoint = Dummy('Panda_waypoint')
        path = arm.get_linear_path(
            waypoint.get_position(), waypoint.get_orientation())
        self.assertIsNotNone(path)

    def test_get_nonlinear_path(self):
        arm = Panda()
        waypoint = Dummy('Panda_waypoint')
        path = arm.get_nonlinear_path(
            waypoint.get_position(), waypoint.get_orientation())
        self.assertIsNotNone(path)

    def test_get_linear_path_and_step(self):
        arm = Panda()
        waypoint = Dummy('Panda_waypoint')
        path = arm.get_linear_path(
            waypoint.get_position(), waypoint.get_orientation())
        self.assertIsNotNone(path)
        done = False
        while not done:
            done = path.step()
            self.pyrep.step()
        self.assertTrue(np.allclose(
            arm.get_tip().get_position(), waypoint.get_position(), atol=0.01))

    def test_get_linear_path_and_get_end(self):
        arm = Panda()
        waypoint = Dummy('Panda_waypoint')
        path = arm.get_linear_path(
            waypoint.get_position(), waypoint.get_orientation())
        path.set_to_end()
        self.assertTrue(np.allclose(
            arm.get_tip().get_position(), waypoint.get_position(), atol=0.001))

    def test_get_linear_path_visualize(self):
        arm = Panda()
        waypoint = Dummy('Panda_waypoint')
        path = arm.get_linear_path(
            waypoint.get_position(), waypoint.get_orientation())
        # Check that it does not error
        path.visualize()

    def test_get_duplicate_arm(self):
        arm = UR3(1)
        self.assertIsInstance(arm, UR3)

    def test_copy_arm(self):
        arm = UR10()
        new_arm = arm.copy()
        self.assertNotEqual(arm, new_arm)


if __name__ == '__main__':
    unittest.main()
