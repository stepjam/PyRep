import unittest
from tests.core import TestCore
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.const import RenderMode, PerspectiveMode


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

    def test_create(self):
        cam = VisionSensor.create([640, 480], perspective_mode=True,
                                  view_angle=35.0,
                                  render_mode=RenderMode.OPENGL3)
        self.assertEqual(cam.capture_rgb().shape, (480, 640, 3))
        self.assertEqual(cam.get_perspective_mode(), PerspectiveMode.PERSPECTIVE)
        self.assertAlmostEqual(cam.get_perspective_angle(), 35.0, 3)
        self.assertEqual(cam.get_render_mode(), RenderMode.OPENGL3)

    def test_get_set_perspective_mode(self):
        for perspective_mode in PerspectiveMode:
            self.cam.set_perspective_mode(perspective_mode)
            self.assertEqual(
                self.cam.get_perspective_mode(),
                perspective_mode
            )

    def test_get_set_render_mode(self):
        for render_mode in [RenderMode.OPENGL, RenderMode.OPENGL3]:
            self.cam.set_render_mode(render_mode)
            self.assertEqual(self.cam.get_render_mode(), render_mode)

    def test_get_set_perspective_angle(self):
        self.cam.set_perspective_angle(45.0)
        self.assertAlmostEqual(self.cam.get_perspective_angle(), 45.0, 3)

    def test_get_set_orthographic_size(self):
        self.cam.set_orthographic_size(1.0)
        self.assertEqual(self.cam.get_orthographic_size(), 1.0)


if __name__ == '__main__':
    unittest.main()
