from typing import Tuple, List
from pyrep.backend import sim
from pyrep.objects.object import Object, object_type_to_class
from pyrep.const import ObjectType


class CartesianPath(Object):
    """An object that defines a cartesian path or trajectory in space.
    """

    @staticmethod
    def create(show_line: bool = True, show_orientation: bool = True,
               show_position: bool = True, closed_path: bool = False,
               automatic_orientation: bool = True, flat_path: bool = False,
               keep_x_up: bool = False) -> 'CartesianPath':
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
            y-axis is not particularly constained). If disabled, the user
            determines the control point's orientation and the Bezier points'
            orientation will be interpolated from the path's control points'
            orientation.
        :param flat_path: If set, then all control points (and subsequently all
            Bezier points) will be constraint to the z=0 plane of the path
            object's local reference frame.
        :param keep_x_up: If set, then the automatic orientation functionality
            will align each Bezier point's z-axis along the path and keep its
            x-axis pointing along the path object's z-axis.

        :return: The newly created cartesian path.
        """
        attributes = 0
        if show_line:
            attributes |= sim.sim_pathproperty_show_line
        if show_orientation:
            attributes |= sim.sim_pathproperty_show_orientation
        if closed_path:
            attributes |= sim.sim_pathproperty_closed_path
        if automatic_orientation:
            attributes |= sim.sim_pathproperty_automatic_orientation
        if flat_path:
            attributes |= sim.sim_pathproperty_flat_path
        if show_position:
            attributes |= sim.sim_pathproperty_show_position
        if keep_x_up:
            attributes |= sim.sim_pathproperty_keep_x_up

        handle = sim.simCreatePath(attributes)
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
        pos = sim.simGetPositionOnPath(self._handle, relative_distance)
        ori = sim.simGetOrientationOnPath(self._handle, relative_distance)
        return pos, ori

    def insert_control_points(self, poses: List[List[float]]) -> None:
        """Inserts one or several control points into the path.

        :param poses: A list of lists containing 6 values representing the pose
            of each of the new control points. Orientation in radians.
        """
        data = []
        for p in poses:
            data.extend(p)
        self._script_call('insertPathControlPoint@PyRep',
                          ints=[self._handle, len(poses)], floats=data)

    def _script_call(self, func: str, ints=(), floats=(), strings=(), bytes=''):
        return sim.simExtCallScriptFunction(
            func, sim.sim_scripttype_addonscript,
            list(ints), list(floats), list(strings), bytes)


object_type_to_class[ObjectType.PATH] = CartesianPath
