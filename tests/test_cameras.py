import unittest
from tests.core import TestCore
from pyrep.const import ObjectType
from pyrep.objects.camera import Camera
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
import numpy as np


class TestCameras(TestCore):

    def setUp(self):
        super().setUp()
        self.camera = Camera('DefaultCamera')
        self.dummy = Dummy('dummy')

    def test_create(self):
        with self.assertRaises(NotImplementedError):
            _ = Camera.create()

    def test_get_set_position(self):
        position = self.camera.get_position()
        self.assertIsInstance(position, np.ndarray)
        self.assertEqual(position.shape, (3,))

        self.camera.set_position([0.1, 0.1, 0.1], self.dummy)
        self.assertTrue(np.allclose(
            self.camera.get_position(self.dummy), [0.1, 0.1, 0.1]))
        self.camera.set_position([0.2, 0.2, 0.2])
        self.assertTrue(np.allclose(self.camera.get_position(), [0.2, 0.2, 0.2]))

    def test_get_object_type(self):
        self.assertEqual(Object.get_object_type('DefaultCamera'),
                         ObjectType.CAMERA)


if __name__ == '__main__':
    unittest.main()
