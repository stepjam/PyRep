import unittest
from tests.core import TestCore
from tests.core import ASSET_DIR
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.force_sensor import ForceSensor
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.errors import WrongObjectTypeError
from os import path
import numpy as np


class TestPyrep(TestCore):

    def test_get_object_wrong_type(self):
        with self.assertRaises(WrongObjectTypeError):
            ProximitySensor('dynamic_cube')

    def test_get_shape(self):
        cube = Shape('dynamic_cube')
        self.assertIsInstance(cube, Shape)

    def test_get_joint(self):
        cube = Joint('prismatic_joint')
        self.assertIsInstance(cube, Joint)

    def test_get_proximity_sensor(self):
        cube = ProximitySensor('proximity_sensor')
        self.assertIsInstance(cube, ProximitySensor)

    def test_get_force_sensor(self):
        cube = ForceSensor('force_sensor')
        self.assertIsInstance(cube, ForceSensor)

    def test_get_cartesian_path(self):
        cube = CartesianPath('cartesian_path')
        self.assertIsInstance(cube, CartesianPath)

    def test_step(self):
        cube = Shape('dynamic_cube')
        start_pos = cube.get_position()
        [self.pyrep.step() for _ in range(2)]
        end_pos = cube.get_position()
        self.assertFalse(np.allclose(start_pos, end_pos))

    def test_load_model(self):
        m = self.pyrep.import_model(path.join(ASSET_DIR, 'loadable_model.ttm'))
        self.assertIsInstance(m, Shape)

    def test_group_objects(self):
        top = Dummy('cubes_under_dummy')
        self.assertEqual(
            len(top.get_objects_in_tree(exclude_base=True)), 3)
        cubes = [Shape('cube%d' % i) for i in range(3)]
        ob = self.pyrep.group_objects(cubes)
        self.assertIsInstance(ob, Object)
        self.assertEqual(
            len(top.get_objects_in_tree(exclude_base=True)), 1)

    def test_merge_objects(self):
        top = Dummy('cubes_under_dummy')
        self.assertEqual(
            len(top.get_objects_in_tree(exclude_base=True)), 3)
        cubes = [Shape('cube%d' % i) for i in range(3)]
        ob = self.pyrep.merge_objects(cubes)
        self.assertIsInstance(ob, Object)
        self.assertEqual(
            len(top.get_objects_in_tree(exclude_base=True)), 1)

    def test_set_configuration_tree(self):
        dynamic_cube = Shape('dynamic_cube')
        pos = dynamic_cube.get_position()
        config = dynamic_cube.get_configuration_tree()
        self.assertIsNotNone(config)
        [self.pyrep.step() for _ in range(10)]
        self.pyrep.set_configuration_tree(config)
        self.assertTrue(np.allclose(pos, dynamic_cube.get_position()))

    def test_create_texture_and_get_texture(self):
        plane, texture = self.pyrep.create_texture(
            path.join(ASSET_DIR, 'wood_texture.jpg'))
        self.assertGreaterEqual(texture.get_texture_id(), 0)
        self.assertEqual(texture.get_texture_id(),
                         plane.get_texture().get_texture_id())


if __name__ == '__main__':
    unittest.main()
