import unittest
import numpy as np
from tests.core import TestCore
from pyrep.objects.light import Light


class TestLights(TestCore):

    def setUp(self):
        super().setUp()
        [self.pyrep.step() for _ in range(10)]
        self.spot_light = Light('spot_light')
        self.omni_light = Light('omni_light')
        self.directed_light = Light('directed_light')
        self.lights = [self.spot_light, self.omni_light, self.directed_light]
        self.light_types = [1, 2, 3]

    # turn on and off

    def test_turn_on_and_off(self):
        for light in self.lights:
            light.turn_on()
            self.assertTrue(light.is_on())
            light.turn_off()
            self.assertTrue(light.is_off())

    # light color

    def test_get_set_light_diffuse(self):
        for light in self.lights:
            orig_diffuse = light.get_diffuse()
            new_diffuse = np.array([0.2, 0.5, 0.9])
            light.set_diffuse(new_diffuse)
            self.assertTrue(np.allclose(light.get_diffuse(), new_diffuse))
            light.set_diffuse(orig_diffuse)
            self.assertTrue(np.allclose(light.get_diffuse(), orig_diffuse))

    def test_get_set_light_specular(self):
        for light in self.lights:
            orig_specular = light.get_specular()
            new_specular = np.array([0.2, 0.5, 0.9])
            light.set_specular(new_specular)
            self.assertTrue(np.allclose(light.get_specular(), new_specular))
            light.set_specular(orig_specular)
            self.assertTrue(np.allclose(light.get_specular(), orig_specular))

    # light intensity properties

    def test_get_set_cast_shadows(self):
        for light in self.lights:
            orig = light.get_intensity_properties()[0]
            new = not orig
            light.set_intensity_properties(cast_shadows=new)
            self.assertTrue(light.get_intensity_properties()[0] is new)
            light.set_intensity_properties(cast_shadows=orig)
            self.assertTrue(np.allclose(light.get_intensity_properties()[0], orig))

    def test_get_set_spot_exponent(self):
        for light in self.lights:
            orig = light.get_intensity_properties()[1]
            new = orig + 2
            light.set_intensity_properties(spot_exponent=new)
            self.assertEqual(light.get_intensity_properties()[1], new)
            light.set_intensity_properties(spot_exponent=orig)
            self.assertEqual(light.get_intensity_properties()[1], orig)

    def test_get_set_spot_cutoff(self):
        for light in self.lights:
            orig = light.get_intensity_properties()[2]
            new = orig - 0.5
            light.set_intensity_properties(spot_cutoff=new)
            self.assertEqual(light.get_intensity_properties()[2], new)
            light.set_intensity_properties(spot_cutoff=orig)
            self.assertEqual(light.get_intensity_properties()[2], orig)

    # ToDo: re-add these tests once attenuation factor setting is supported in CoppeliaSim.
    #  setObjectFloatParams() does not change the properties of the light even with in-scene lua code.
    '''
    def test_get_set_const_atten_factor(self):
        for light in self.lights:
            orig = light.get_intensity_properties()[3]
            new = orig + 1.5
            light.set_intensity_properties(const_atten_factor=new)
            assert light.get_intensity_properties()[3] == new
            light.set_intensity_properties(const_atten_factor=orig)
            assert light.get_intensity_properties()[3] == orig

    def test_get_set_linear_atten_factor(self):
        for light in self.lights:
            orig = light.get_intensity_properties()[4]
            new = orig + 1.5
            light.set_intensity_properties(linear_atten_factor=new)
            assert light.get_intensity_properties()[4] == new
            light.set_intensity_properties(linear_atten_factor=orig)
            assert light.get_intensity_properties()[4] == orig

    def test_get_set_quad_atten_factor(self):
        for light in self.lights:
            orig = light.get_intensity_properties()[5]
            new = orig + 1.5
            light.set_intensity_properties(quad_atten_factor=new)
            assert light.get_intensity_properties()[5] == new
            light.set_intensity_properties(quad_atten_factor=orig)
            assert light.get_intensity_properties()[5] == orig
    '''


if __name__ == '__main__':
    unittest.main()
