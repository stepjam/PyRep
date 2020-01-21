from pyrep.objects.object import Object
from pyrep.const import ObjectType
from pyrep.backend import sim
from typing import List, Optional

class Octree(Object):
    """An octree object."""

    @staticmethod
    def create(voxel_size: float, point_size: Optional[float] = None,
               options: int = 0) -> 'Octree':
        """Creates an octree object and inserts in the scene.
        
        :param voxelSize: The resolution of octree voxels.
        :param options: Octree options.
        :param pointSize: Point representation size of voxels.
        :return: The newly created Octree.
        """
        if point_size is None:
            point_size = voxel_size
        handle = sim.simCreateOctree(voxel_size, options, point_size)
        return Octree(handle)

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.OCTREE

    def insert_voxels(self, points: List[float], color: Optional[float] = None,
                      options: int = 0) -> None:
        """Inserts voxels into the octree.
        
        :param points: A list of x,y,z numbers.
        :param color: A list containing RGB data, or None.
        :param options: Voxel insertion options.
        """
        if not isinstance(points, list):
            raise ValueError(
                'Octree.insert_voxels: points parameter is not a list.')
        if len(points) % 3 is not 0:
            raise ValueError(
                'Octree.insert_voxels: points parameter length '
                'not a multiple of 3.')
        if color is not None:
            if not isinstance(color, list):
                raise ValueError(
                    'Octree.insert_voxels: color parameter not a list.')
            elif len(color) is not 3:
                raise ValueError(
                    'Octree.insert_voxels: color parameter not an RGB list.')
        sim.simInsertVoxelsIntoOctree(self._handle, options, points, color,None) 
        return

    def remove_voxels(self, points : Optional[List[float]],
                      options: int = 0) -> None:
        """Remove voxels from the octree.
        
        :param points: A list of x,y,z numbers, or None to clear the octree.
        :param options: Voxel removal options.
        """
        if points is not None:
            if not isinstance(points, list):
                raise ValueError(
                    'Octree.insert_voxels: points parameter is not a list.')
            if len(points) % 3 is not 0:
                raise ValueError(
                    'Octree.insert_voxels: points parameter length '
                    'not a multiple of 3.')
        sim.simRemoveVoxelsFromOctree(self._handle, options, points)

    def get_voxels(self) -> list:
        """Returns voxels from the octree.
        
        :return: List of voxel x,y,z coordinates.
        """
        return sim.simGetOctreeVoxels(self._handle)

    def insert_object(self, obj: Object, color: Optional[float] = None,
                      options: int = 0) -> None:
        """Inserts object into the octree.
        
        :param obj: Object to insert.
        :param color: A list containing RGB data, or None.
        :param options: Object insertion options.
        """
        if color is not None:
            if not isinstance(color, list):
                raise ValueError(
                    'Octree.insert_object: color parameter not a list.')
            elif len(color) is not 3:
                raise ValueError(
                    'Octree.insert_object: color parameter not an RGB list.')
        sim.simInsertObjectIntoOctree(self._handle, obj.get_handle(), options,
            color, 0)
        return

    def subtract_object(self, obj: Object, options: int = 0) -> None:
        """Subtract object from the octree.
        
        :param obj: Object to subtract.
        :param options: Object subtraction options.
        """
        sim.simSubtractObjectFromOctree(self._handle, obj.get_handle(), options)
        return

    def check_point_occupancy(self, points: List[float],
                              options: int = 0) -> bool:
        if not isinstance(points, list):
            raise ValueError(
                'Octree.check_point_occupancy: points parameter is not a list.')
        if len(points) % 3 is not 0:
            raise ValueError(
                'Octree._check_point_occupancy: points parameter length '
                'not a multiple of 3.')
        return sim.simCheckOctreePointOccupancy(self._handle, options, points)

    def clear_voxels(self) -> None:
        """Clears all voxels from the octree.
        """
        sim.simRemoveVoxelsFromOctree(self._handle, 0, None)
