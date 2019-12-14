import unittest
from tests.core import TestCore
from pyrep.sensors.accelerometer import Accelerometer


class TestAccelerometer(TestCore):

    def setUp(self):
        super().setUp()
        self.sensor = Accelerometer('accelerometer')

    def test_read(self):
        accelerations = self.sensor.read()
        self.assertEqual(len(accelerations), 3)


if __name__ == '__main__':
    unittest.main()
