import unittest
from tests.core import TestCore
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape, TextureMappingMode
from tests.core import ASSET_DIR
from os import path
import numpy as np


class TestShapes(TestCore):

    def setUp(self):
        super().setUp()
        self.dynamic_cube = Shape('dynamic_cube')

    def test_create_primitive_simple(self):
        pr = Shape.create(
            PrimitiveShape.CUBOID, size=[1., 1., 1.])
        self.assertIsInstance(pr, Shape)

    def test_create_primitive_complex(self):
        pr = Shape.create(
            PrimitiveShape.CUBOID, size=[1., 1., 1.], mass=2.0,
            smooth=True, respondable=True, static=False,
            position=[1.1, 1.2, 1.3], orientation=[0.1, 0.2, 0.3],
            color=[0.7, 0.8, 0.9])
        self.assertIsInstance(pr, Shape)
        self.assertTrue(np.allclose(pr.get_position(), [1.1, 1.2, 1.3]))
        self.assertTrue(np.allclose(pr.get_orientation(), [0.1, 0.2, 0.3]))
        self.assertTrue(np.allclose(pr.get_color(), [0.7, 0.8, 0.9]))

    def test_import_shape(self):
        ob = Shape.import_shape(
            path.join(ASSET_DIR, 'cracker_box/textured_simple.obj'))
        self.assertIsInstance(ob, Shape)

    def test_import_mesh(self):
        ob = Shape.import_mesh(
            path.join(ASSET_DIR, 'test_mesh_bowl.obj'))
        self.assertIsInstance(ob, Shape)

    def test_create_mesh(self):
        ob = Shape.create_mesh(
            vertices=[-0.1, -0.1, 0.0,
                      -0.1, 0.1, 0.0,
                      0.1, 0.0, 0.0], indices=[0, 1, 2])
        self.assertIsInstance(ob, Shape)

    def test_convex_decompose(self):
        ob = Shape.import_mesh(
            path.join(ASSET_DIR, 'test_mesh_bowl.obj'))
        self.assertIsInstance(ob, Shape)
        cd_1 = ob.get_convex_decomposition()
        self.assertIsInstance(cd_1, Shape)
        self.assertNotEqual(ob, cd_1)
        cd_2 = ob.get_convex_decomposition(morph=True)
        self.assertIsInstance(cd_2, Shape)
        self.assertEqual(ob, cd_2)

    def test_get_set_color(self):
        self.dynamic_cube.set_color([.5] * 3)
        self.assertEqual(self.dynamic_cube.get_color(), [.5] * 3)

    def test_get_set_transparency(self):
        self.dynamic_cube.set_transparency(0.6)
        self.assertAlmostEqual(self.dynamic_cube.get_transparency(), 0.6)

    def test_get_set_mass(self):
        self.dynamic_cube.set_mass(3.5)
        self.assertEqual(self.dynamic_cube.get_mass(), 3.5)

    def test_get_set_respondable(self):
        self.dynamic_cube.set_respondable(False)
        self.assertFalse(self.dynamic_cube.is_respondable())
        self.dynamic_cube.set_respondable(True)
        self.assertTrue(self.dynamic_cube.is_respondable())

    def test_get_set_dynamic(self):
        self.dynamic_cube.set_dynamic(False)
        self.assertFalse(self.dynamic_cube.is_dynamic())
        self.dynamic_cube.set_dynamic(True)
        self.assertTrue(self.dynamic_cube.is_dynamic())

    def test_get_mesh_data(self):
        vertices, indices, normals = self.dynamic_cube.get_mesh_data()
        n_vertices = 8
        n_faces = 12
        self.assertEqual(vertices.shape, (n_vertices, 3))
        self.assertEqual(indices.shape, (n_faces, 3))
        self.assertEqual(normals.shape, (n_faces * 3, 3))

    def test_set_texture(self):
        _, texture = self.pyrep.create_texture(
            path.join(ASSET_DIR, 'wood_texture.jpg'))
        self.dynamic_cube.set_texture(texture, TextureMappingMode.CUBE)
        self.assertEqual(texture.get_texture_id(),
                         self.dynamic_cube.get_texture().get_texture_id())

    def test_get_shape_viz(self):
        visual = Shape('cracker_box_visual')
        info = visual.get_shape_viz(index=0)
        self.assertIsInstance(info.vertices, np.ndarray)
        self.assertEqual(info.vertices.shape[1], 3)
        self.assertIsInstance(info.indices, np.ndarray)
        self.assertEqual(info.indices.shape[1], 3)
        self.assertIsInstance(info.normals, np.ndarray)
        self.assertEqual(info.normals.shape[1], 3)
        self.assertIsInstance(info.shading_angle, float)
        self.assertIsInstance(info.colors, np.ndarray)
        self.assertTupleEqual(info.colors.shape, (9,))
        self.assertIsInstance(info.texture, np.ndarray)
        self.assertTupleEqual(info.texture.shape, (512, 512, 4))
        self.assertIsInstance(info.texture_id, int)
        self.assertIsInstance(info.texture_coords, np.ndarray)
        self.assertEqual(info.texture_coords.shape[1], 2)
        self.assertIsInstance(info.texture_apply_mode, int)
        self.assertIsInstance(info.texture_options, int)

    def test_apply_texture(self):
        visual = Shape('cracker_box_visual')
        info = visual.get_shape_viz(index=0)
        self.assertNotEqual(info.texture_coords.size, 0)
        self.assertNotEqual(info.texture.size, 0)
        texture = info.texture[:, :, [2, 1, 0, 3]]  # rgba -> bgra
        visual.apply_texture(info.texture_coords, texture, is_rgba=True)

    def test_decimate_mesh(self):
        visual = Shape('cracker_box_visual')
        _, old_indices, _ = visual.get_mesh_data()
        new_mesh = visual.decimate_mesh(0.2)
        _, new_indices, _ = new_mesh.get_mesh_data()
        self.assertLess(len(new_indices), len(old_indices) * 0.3)

    def test_compute_mass_and_inertia(self):
        before = self.dynamic_cube.get_mass()
        self.dynamic_cube.compute_mass_and_inertia(2000)
        after = self.dynamic_cube.get_mass()
        self.assertNotEqual(before, after)

    def test_add_force(self):
        before = self.dynamic_cube.get_position()
        self.dynamic_cube.add_force([0, 0, 0], [100, 100, 100])
        self.pyrep.step()
        after = self.dynamic_cube.get_position()
        self.assertTrue(np.all(before != after))

    def test_add_force_and_torque(self):
        before = self.dynamic_cube.get_position()
        self.dynamic_cube.add_force_and_torque([100, 100, 100], [100, 100, 100])
        self.pyrep.step()
        after = self.dynamic_cube.get_position()
        self.assertTrue(np.all(before != after))


if __name__ == '__main__':
    unittest.main()
