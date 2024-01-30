import unittest
from pyrep.errors import PyRepError
from tests.core import TestCore
from pyrep.misc.signals import IntegerSignal, FloatSignal, StringSignal


class TestSignal(TestCore):

    SIGNALS = [
        (IntegerSignal, 99),
        (FloatSignal, 55.3),
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
                sig.clear()

    def test_get_signal_fails_when_empty(self):
        for signal_class, test_value in TestSignal.SIGNALS:
            with self.subTest(signal=str(signal_class)):
                sig = signal_class('my_signal')
                self.assertIsNone(sig.get())


if __name__ == '__main__':
    unittest.main()
