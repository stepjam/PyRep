import unittest
from tests.core import TestCore
from pyrep.const import RenderMode
from pyrep.sensors.spherical_vision_sensor import SphericalVisionSensor


class TestSphericalVisionSensors(TestCore):

    def setUp(self):
        super().setUp()
        [self.pyrep.step() for _ in range(10)]
        self.spherical_vision_sensor = SphericalVisionSensor('sphericalVisionRGBAndDepth')

    def test_handle_explicitly(self):
        # non-blank image
        self.spherical_vision_sensor.handle_explicitly()
        rgb = self.spherical_vision_sensor.capture_rgb()
        self.assertNotEqual(rgb.sum(), 0)

    def test_capture_rgb(self):
        img = self.spherical_vision_sensor.capture_rgb()
        self.assertEqual(img.shape, (256, 512, 3))
        #  Check that it's not a blank image
        self.assertFalse(img.min() == img.max() == 0.0)

    def test_capture_depth(self):
        img = self.spherical_vision_sensor.capture_depth()
        self.assertEqual(img.shape, (256, 512))
        # Check that it's not a blank depth map
        self.assertFalse(img.min() == img.max() == 1.0)

    def test_get_set_resolution(self):
        self.spherical_vision_sensor.set_resolution([480, 240])
        self.assertEqual(self.spherical_vision_sensor.get_resolution(), [480, 240])
        self.assertEqual(self.spherical_vision_sensor.capture_rgb().shape, (240, 480, 3))

    def test_get_set_render_mode(self):
        for render_mode in [RenderMode.OPENGL, RenderMode.OPENGL3]:
            self.spherical_vision_sensor.set_render_mode(render_mode)
            self.assertEqual(self.spherical_vision_sensor.get_render_mode(), render_mode)

    def test_get_set_near_clipping_plane(self):
        self.spherical_vision_sensor.set_near_clipping_plane(0.1)
        self.assertAlmostEqual(self.spherical_vision_sensor.get_near_clipping_plane(), 0.1)

    def test_get_set_far_clipping_plane(self):
        self.spherical_vision_sensor.set_far_clipping_plane(0.1)
        self.assertAlmostEqual(self.spherical_vision_sensor.get_far_clipping_plane(), 0.1)

    def test_get_set_windowed_size(self):
        self.spherical_vision_sensor.set_windowed_size((640, 480))
        self.assertEqual(self.spherical_vision_sensor.get_windowed_size(), (640, 480))

    def test_get_set_entity_to_render(self):
        self.spherical_vision_sensor.set_entity_to_render(-1)
        self.assertEqual(self.spherical_vision_sensor.get_entity_to_render(), -1)


if __name__ == '__main__':
    unittest.main()
