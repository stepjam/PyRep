import unittest
from core import TestCore
from pyrep import PyRep
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.objects.dummy import Dummy
import numpy as np
from os import path

from pyrep.robots.mobiles.youBot import youBot
from pyrep.robots.mobiles.turtlebot import turtlebot
from pyrep.robots.mobiles.LineTracer import LineTracer

ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets')


MOBILES = [
    ('youBot', youBot),
    ('LineTracer', LineTracer),
    ('turtlebot', turtlebot),
]


class TestMobilesAndConfigurationPaths(TestCore):

    def setUp(self):
        self.pyrep = PyRep()
        self.pyrep.launch(path.join(
            ASSET_DIR, 'test_scene_mobiles.ttt'), headless=True)
        self.pyrep.step()
        self.pyrep.start()

    # It is enough to only test the constructor of each mobile (in there we make
    # assumptions about the structure of the mobile model). All other tests
    # can be run on one mobile.
    def test_get_mobile(self):
        for mobile_name, mobile_type in MOBILES:
            with self.subTest(mobile=mobile_name):
                mobile = mobile_type()
                self.assertIsInstance(mobile, mobile_type)

    def test_get_linear_path(self):
        mobile = youBot()
        waypoint = Dummy('youBot_waypoint')
        path = mobile.get_linear_path(
            waypoint.get_position(), waypoint.get_orientation()[-1])
        self.assertIsNotNone(path)

    def test_get_nonlinear_path(self):
        mobile = youBot()
        waypoint = Dummy('youBot_waypoint')
        path = mobile.get_nonlinear_path(
            waypoint.get_position(), waypoint.get_orientation()[-1])
        self.assertIsNotNone(path)

    def test_get_linear_path_and_step(self):
        mobile = youBot()
        waypoint = Dummy('youBot_waypoint')
        path = mobile.get_linear_path(
            waypoint.get_position(), waypoint.get_orientation()[-1])
        self.assertIsNotNone(path)
        done = False
        while not done:
            _, done = path.step()
            self.pyrep.step()
        self.assertTrue(np.allclose(
            mobile.get_tip().get_position()[:2], waypoint.get_position()[:2], atol=0.001))

    def test_get_linear_path_and_get_end(self):
        mobile = youBot()
        waypoint = Dummy('youBot_waypoint')
        path = mobile.get_linear_path(
            waypoint.get_position(), waypoint.get_orientation()[-1])
        path.set_to_end()
        self.assertTrue(np.allclose(
            mobile.get_tip().get_position()[:2], waypoint.get_position()[:2], atol=0.001))

    def test_get_linear_path_visualize(self):
        mobile = youBot()
        waypoint = Dummy('youBot_waypoint')
        path = mobile.get_linear_path(
            waypoint.get_position(), waypoint.get_orientation()[-1])
        # Check that it does not error
        path.visualize()

    def test_get_duplicate_mobile(self):
        mobile = youBot(1)
        self.assertIsInstance(mobile, youBot)

    def test_copy_mobile(self):
        mobile = LineTracer()
        new_mobile = mobile.copy()
        self.assertNotEqual(mobile, new_mobile)


if __name__ == '__main__':
    unittest.main()
