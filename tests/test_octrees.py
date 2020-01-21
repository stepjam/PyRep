import unittest

from pyrep.const import ObjectType

from tests.core import TestCore
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from pyrep.objects.octree import Octree

class TestOctrees(TestCore):

    def setUp(self):
        super().setUp()
        self.octree = Octree.create(0.025)
        self.shape = Shape('Panda_link0_visual')

    def test_octree_insert_and_remove_voxels(self):
        point = [0.0, 0.0, 0.0]
        self.octree.insert_voxels(point)
        voxels = self.octree.get_voxels()
        # As many as 8 voxels may be added from a single point insertion.
        self.assertTrue(1 <= len(voxels)//3 and len(voxels)/3 <= 8)
        self.assertTrue(self.octree.check_point_occupancy(point))
        self.octree.remove_voxels(point)
        voxels = self.octree.get_voxels()
        self.assertTrue(len(voxels)//3 is 0)
        self.assertFalse(self.octree.check_point_occupancy(point))

    def test_octree_insert_and_subtract_object(self):
        self.octree.insert_object(self.shape)
        voxels = self.octree.get_voxels()
        self.assertTrue(1 <= len(voxels)//3)
        self.octree.subtract_object(self.shape)
        voxels = self.octree.get_voxels()
        self.assertTrue(len(voxels)//3 is 0)

    def test_octree_insert_and_clear(self):
        self.octree.insert_object(self.shape)
        voxels = self.octree.get_voxels()
        self.assertTrue(1 <= len(voxels)//3)
        self.octree.clear_voxels()
        voxels = self.octree.get_voxels()
        self.assertTrue(len(voxels)//3 is 0)
