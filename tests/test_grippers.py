import unittest

from pyrep.robots.end_effectors.robotiq85_gripper import Robotiq85Gripper
from tests.core import TestCore
from pyrep import PyRep
from os import path

from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.robots.end_effectors.mico_gripper import MicoGripper
from pyrep.robots.end_effectors.jaco_gripper import JacoGripper
from pyrep.robots.end_effectors.baxter_gripper import BaxterGripper

ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets')

GRIPPERS = [
    ('PandaGripper', PandaGripper, 0.04),
    ('BaxterGripper', BaxterGripper, 0.04),
    ('MicoGripper', MicoGripper, 0.2),
    ('JacoGripper', JacoGripper, 0.2),
    ('Robotiq85Gripper', Robotiq85Gripper, 0.04),
]


class TestArmsAndConfigurationPaths(TestCore):

    def setUp(self):
        self.pyrep = PyRep()
        self.pyrep.launch(path.join(
            ASSET_DIR, 'test_scene_robots.ttt'), headless=True)
        self.pyrep.step()
        self.pyrep.start()

    def test_get_gripper(self):
        for gripper_name, gripper_type, vel in GRIPPERS:
            with self.subTest(gripper=gripper_name):
                gripper = gripper_type()
                self.assertIsInstance(gripper, gripper_type)

    def test_close_open_gripper(self):
        for gripper_name, gripper_type, vel in GRIPPERS:
            with self.subTest(gripper=gripper_name):
                gripper = gripper_type()
                self.pyrep.step()
                done = False
                i = 0
                while not done:
                    done = gripper.actuate(0.0, velocity=vel)
                    self.pyrep.step()
                    i += 1
                    if i > 1000:
                        self.fail('Took too many steps to close')
                done = False
                i = 0
                open_amount = 1.0 if gripper_name == 'Robotiq85Gripper' else 0.8
                while not done:
                    done = gripper.actuate(open_amount, velocity=vel)
                    self.pyrep.step()
                    i += 1
                    if i > 1000:
                        self.fail('Took too many steps to open')
                self.assertAlmostEqual(
                    gripper.get_open_amount()[0], open_amount, delta=0.05)

    def test_get_duplicate_gripper(self):
        g = BaxterGripper(1)
        self.assertIsInstance(g, BaxterGripper)

    def test_copy_gripper(self):
        g = JacoGripper()
        new_g = g.copy()
        self.assertNotEqual(g, new_g)


if __name__ == '__main__':
    unittest.main()
