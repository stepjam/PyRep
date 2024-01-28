from typing import Tuple, List

import numpy as np

from pyrep.backend import sim_const as simc
from pyrep.backend.sim import SimBackend
from pyrep.objects.object import Object, object_type_to_class
from pyrep.const import ObjectType


class CartesianPath(Object):
    """An object that defines a cartesian path or trajectory in space.
    """

    @staticmethod
    def create(show_line: bool = True, show_orientation: bool = True,
               show_position: bool = True, closed_path: bool = False,
               automatic_orientation: bool = True, flat_path: bool = False,
               keep_x_up: bool = False, line_size: int = 1,
               length_calculation_method: int = simc.sim_distcalcmethod_dl_if_nonzero, control_point_size: float = 0.01,
               ang_to_lin_conv_coeff: float = 1., virt_dist_scale_factor: float = 1.,
               path_color: tuple = (0.1, 0.75, 1.), paths_points: list = []) -> 'CartesianPath':
        """Creates a cartesian path and inserts in the scene.

        :param show_line: Shows line in UI.
        :param show_position: Shows line in UI.
        :param show_orientation: Shows orientation in UI.
        :param closed_path: If set, then a path's last control point will be
            linked to its first control point to close the path and make its
            operation cyclic. A minimum of 3 control points are required for
            a path to be closed.
        :param automatic_orientation: If set, then all control points and
            Bezier point's orientation will automatically be calculated in
            order to have a point's z-axis along the path, and its y-axis
            pointing outwards its curvature (if keep x up is enabled, the
            y-axis is not particularly constrained). If disabled, the user
            determines the control point's orientation and the Bezier points'
            orientation will be interpolated from the path's control points'
            orientation.
        :param flat_path: If set, then all control points (and subsequently all
            Bezier points) will be constraint to the z=0 plane of the path
            object's local reference frame.
        :param keep_x_up: If set, then the automatic orientation functionality
            will align each Bezier point's z-axis along the path and keep its
            x-axis pointing along the path object's z-axis.
        :param line_size: Size of the line in pixels.
        :param length_calculation_method: Method for calculating the path length. See
            https://www.coppeliarobotics.com/helpFiles/en/apiConstants.htm#distanceCalculationMethods
        :param control_point_size: Size of the control points in the path.
        :param ang_to_lin_conv_coeff: The angular to linear conversion coefficient.
        :param virt_dist_scale_factor: The virtual distance scaling factor.
        :param path_color: Ambient diffuse rgb color of the path.

        :return: The newly created cartesian path.
        """
        attributes = 16  #  the path points' orientation is computed according to the orientationMode below
        if closed_path:
            attributes |= 2
        orientation_mode = 0
        if automatic_orientation:
            if keep_x_up:
                orientation_mode |= 4
            else:
                orientation_mode |= 5
        sim_api = SimBackend().sim_api
        subdiv = 100
        smoothness = 1.0
        up_vector = [0, 0, 1]
        if len(paths_points) == 0:
            #  Path must have at least 2 points
            paths_points = np.zeros((14,)).tolist()
        handle = sim_api.createPath(paths_points, attributes, subdiv, smoothness, orientation_mode, up_vector)
        return CartesianPath(handle)

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.PATH

    def get_pose_on_path(self, relative_distance: float
                         ) -> Tuple[List[float], List[float]]:
        """Retrieves the absolute interpolated pose of a point along the path.

        :param relative_distance: A value between 0 and 1, where 0 is the
            beginning of the path, and 1 the end of the path.
        :return: A tuple containing the x, y, z position, and the x, y, z
            orientation of the point on the path (in radians).
        """
        sim_api = SimBackend().sim_api
        path_data = sim_api.unpackDoubleTable(sim_api.readCustomDataBlock(self.get_handle(), 'PATH'))
        m = np.array(path_data).reshape(len(path_data) // 7, 7)
        path_positions = m[:, :3].flatten().tolist()
        path_quaternions = m[:, 3:].flatten().tolist()
        path_lengths, total_length = sim_api.getPathLengths(path_positions, 3)
        pos = sim_api.getPathInterpolatedConfig(path_positions, path_lengths, total_length * relative_distance)
        quat = sim_api.getPathInterpolatedConfig(path_quaternions, path_lengths, total_length * relative_distance, None, [2, 2, 2, 2])
        # TODO: Hack for now; convert quat -> euler using library
        from pyrep.objects import Dummy
        d = Dummy.create()
        d.set_position(pos, self)
        d.set_quaternion(quat, self)
        # To word coords
        pos = d.get_position()
        ori = d.get_orientation()
        d.remove()
        return pos, ori


object_type_to_class[ObjectType.PATH] = CartesianPath
