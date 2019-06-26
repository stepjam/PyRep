import unittest
from tests.core import TestCore
from pyrep.objects.vision_sensor import VisionSensor


class TestVisionSensors(TestCore):

    def setUp(self):
        super().setUp()
        [self.pyrep.step() for _ in range(10)]
        self.cam = VisionSensor('cam0')

    def test_capture_rgb(self):
        img = self.cam.capture_rgb()
        self.assertEqual(img.shape, (16, 16, 3))
        #  Check that it's not a blank image
        self.assertFalse(img.min() == img.max() == 0.0)

    def test_capture_depth(self):
        img = self.cam.capture_depth()
        self.assertEqual(img.shape, (16, 16))
        # Check that it's not a blank depth map
        self.assertFalse(img.min() == img.max() == 1.0)


if __name__ == '__main__':
    unittest.main()
