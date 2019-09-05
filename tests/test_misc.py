import unittest
from pyrep.errors import PyRepError
from tests.core import TestCore
from pyrep.misc.distance import Distance


class TestMisc(TestCore):

    def test_get_distance(self):
        Distance('dist_cubes')

    def test_read_distance(self):
        d = Distance('dist_cubes')
        dist = d.read()
        self.assertAlmostEqual(dist, 0.1, places=3)

    def test_read_distance_not_measurable(self):
        d = Distance('dist_cubes_fail')
        with self.assertRaises(PyRepError):
            d.read()


if __name__ == '__main__':
    unittest.main()
