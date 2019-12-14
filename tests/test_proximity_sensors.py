import unittest
from tests.core import TestCore
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape


class TestProximitySensors(TestCore):

    def setUp(self):
        super().setUp()
        self.sensor = ProximitySensor('proximity_sensor')

    def test_read(self):
        self.pyrep.step()
        distance = self.sensor.read()
        self.pyrep.step()
        self.assertAlmostEqual(distance, 0.1)

    def test_is_detected(self):
        ob1 = Shape('simple_model')
        ob2 = Shape('dynamic_cube')
        self.assertTrue(self.sensor.is_detected(ob1))
        self.assertFalse(self.sensor.is_detected(ob2))


if __name__ == '__main__':
    unittest.main()
