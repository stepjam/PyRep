import unittest
from tests.core import TestCore
from pyrep import PyRep
from pyrep.objects.shape import Shape
from os import path

from pyrep.robots.end_effectors.dobot_suction_cup import DobotSuctionCup
from pyrep.robots.end_effectors.baxter_suction_cup import BaxterSuctionCup

ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets')

SUCTION_CUPS = [
    ('DobotSuctionCup', DobotSuctionCup),
    ('BaxterSuctionCup', BaxterSuctionCup),
]


class TestSuctionCups(TestCore):

    def setUp(self):
        self.pyrep = PyRep()
        self.pyrep.launch(path.join(
            ASSET_DIR, 'test_scene_robots.ttt'), headless=True)
        self.pyrep.step()
        self.pyrep.start()

    def test_get_suction_cup(self):
        for cup_name, cup_type in SUCTION_CUPS:
            with self.subTest(suction_cup=cup_name):
                cup = cup_type()
                self.assertIsInstance(cup, cup_type)

    def test_grasp_and_release_with_suction(self):
        for cup_name, cup_type in SUCTION_CUPS:
            with self.subTest(suction_cup=cup_name):
                suction_cube = Shape('%s_cube' % cup_name)
                cube = Shape('cube')
                cup = cup_type()
                self.pyrep.step()
                grasped = cup.grasp(cube)
                self.assertFalse(grasped)
                self.assertEqual(len(cup.get_grasped_objects()), 0)
                grasped = cup.grasp(suction_cube)
                self.assertTrue(grasped)
                self.assertListEqual(cup.get_grasped_objects(), [suction_cube])
                cup.release()
                self.assertEqual(len(cup.get_grasped_objects()), 0)


if __name__ == '__main__':
    unittest.main()
