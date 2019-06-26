import unittest
from tests.core import TestCore
from pyrep.objects.cartesian_path import CartesianPath


class TestCartesianPaths(TestCore):

    def setUp(self):
        super().setUp()
        self.cart_path = CartesianPath('cartesian_path')

    def test_create_cartesian_path(self):
        p = CartesianPath.create()
        self.assertIsInstance(p, CartesianPath)

    def test_get_pose_on_path(self):
        pos, ori = self.cart_path.get_pose_on_path(0.5)
        self.assertEqual(len(pos), 3)
        self.assertEqual(len(ori), 3)

    def test_insert_control_points(self):
        points = [[0.1] * 6]
        # Just check that it does not throw an exception.
        self.cart_path.insert_control_points(points)


if __name__ == '__main__':
    unittest.main()
