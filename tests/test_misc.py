import unittest
from pyrep.errors import PyRepError
from tests.core import TestCore
from pyrep.misc.distance import Distance
from pyrep.misc.signals import IntegerSignal, FloatSignal
from pyrep.misc.signals import DoubleSignal, StringSignal


# TODO: These tests will be re-enabled once bug has been fixed in CoppeliaSim.
class TestMisc(TestCore):
    pass

    # def test_get_distance(self):
        # Distance('dist_cubes')

    # def test_read_distance(self):
        # d = Distance('dist_cubes')
        # dist = d.read()
        # self.assertAlmostEqual(dist, 0.1, places=3)

    # def test_read_distance_not_measurable(self):
        # d = Distance('dist_cubes_fail')
        # with self.assertRaises(PyRepError):
        #     d.read()


class TestSignal(TestCore):

    SIGNALS = [
        (IntegerSignal, 99),
        (FloatSignal, 55.3),
        (DoubleSignal, 22.2),
        (StringSignal, 'hello')
    ]

    def test_set_get_clear_signals(self):
        for signal_class, test_value in TestSignal.SIGNALS:
            with self.subTest(signal=str(signal_class)):
                sig = signal_class('my_signal')
                sig.set(test_value)
                ret_value = sig.get()
                if isinstance(test_value, float):
                    self.assertAlmostEqual(ret_value, test_value, places=3)
                else:
                    self.assertEqual(ret_value, test_value)
                clears = sig.clear()
                self.assertEqual(clears, 1)

    def test_get_signal_fails_when_empty(self):
        for signal_class, test_value in TestSignal.SIGNALS:
            with self.subTest(signal=str(signal_class)):
                sig = signal_class('my_signal')
                with self.assertRaises(PyRepError):
                    sig.get()


if __name__ == '__main__':
    unittest.main()
