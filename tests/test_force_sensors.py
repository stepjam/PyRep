import unittest
from tests.core import TestCore
from pyrep.objects.force_sensor import ForceSensor


class TestForceSensors(TestCore):

    def setUp(self):
        super().setUp()
        self.sensor = ForceSensor('force_sensor')

    def test_read(self):
        force_vector, torque_vector = self.sensor.read()
        self.assertEqual(len(force_vector), 3)
        self.assertEqual(len(torque_vector), 3)

    def test_create(self):
        sensor = ForceSensor.create()
        sensor.remove()


if __name__ == '__main__':
    unittest.main()
