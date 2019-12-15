import unittest
from tests.core import TestCore
from pyrep.sensors.gyroscope import Gyroscope


class TestGyroscope(TestCore):

    def setUp(self):
        super().setUp()
        self.sensor = Gyroscope('gyroscope')

    def test_read(self):
        angular_velocities = self.sensor.read()
        self.assertEqual(len(angular_velocities), 3)


if __name__ == '__main__':
    unittest.main()
