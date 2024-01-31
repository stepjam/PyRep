import unittest

import numpy as np

from tests.core import TestCore
from pyrep.objects.cartesian_path import CartesianPath


class TestCartesianPaths(TestCore):
    def test_create_cartesian_path(self):
        p = CartesianPath.create()
        self.assertIsInstance(p, CartesianPath)

    def test_get_pose_on_path(self):
        p = CartesianPath.create(
            paths_points=[0, 0, 0, 0, 0, 0, 1, 0.1, 0.1, 0.2, 0, 0, 0, 1]
        )
        pos, ori = p.get_pose_on_path(0.5)
        self.assertEqual(len(pos), 3)
        self.assertEqual(len(ori), 3)
        self.assertTrue(np.allclose(pos, [0.05, 0.05, 0.1]))


if __name__ == "__main__":
    unittest.main()
