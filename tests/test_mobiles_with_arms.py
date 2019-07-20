import unittest
from tests.core import TestCore
from pyrep import PyRep
from os import path

from pyrep.robots.mobiles.locobot import LoCoBot
from pyrep.robots.arms.locobot_arm import LoCoBotArm

ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets')

# (Name, (Base, Arm))
ROBOTS = [
    ('LoCoBot', (LoCoBot, LoCoBotArm)),
]


class TestMobilesWithArms(TestCore):
    """Used for testing mobile bases with arms.
    """

    def setUp(self):
        self.pyrep = PyRep()
        self.pyrep.launch(path.join(
            ASSET_DIR, 'test_scene_mobiles_with_arms.ttt'), headless=True)
        self.pyrep.step()
        self.pyrep.start()

    # It is enough to only test the constructor of each mobile (in there we make
    # assumptions about the structure of the mobile model). All other tests
    # can be run on one mobile.
    def test_get_mobile(self):
        for mobile_name, (mobile_type, arm_type) in ROBOTS:
            with self.subTest(mobile=mobile_name):
                mobile = mobile_type()
                arm = arm_type()
                self.assertIsInstance(mobile, mobile_type)
                self.assertIsInstance(arm, arm_type)


if __name__ == '__main__':
    unittest.main()
