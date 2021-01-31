from pyrep.backend import sim, utils
from pyrep.robots.configuration_paths.configuration_path import (
    ConfigurationPath)
from pyrep.robots.mobiles.mobile_base import MobileBase
from pyrep.const import PYREP_SCRIPT_TYPE
from math import sqrt
from typing import List


class MobileConfigurationPath(ConfigurationPath):
    """A path expressed in joint configuration space.

    Paths are retrieved from an :py:class:`Mobile`, and are associated with the
    mobile base that generated the path.

    This class is used for executing motion along a path via the
    _get_base_actuation function employing a proportional controller.
    """

    def __init__(self, mobile: MobileBase, path_points: List[List[float]]):
        self._mobile = mobile
        self._path_points = path_points
        self._drawing_handle = None
        self._path_done = False
        self.i_path = -1
        self.inter_done = True
        self._num_joints = mobile.get_joint_count()

        self.set_to_start()

        if len(self._path_points) > 2:
            self._set_inter_target(0)

    def step(self) -> bool:
        """Make a step along the trajectory.

        Step forward by calling _get_base_actuation to get the velocity needed
        to be applied at the wheels.

        NOTE: This does not step the physics engine. This is left to the user.

        :return: If the end of the trajectory has been reached.

        """
        raise NotImplementedError()

    def set_to_start(self) -> None:
        """Sets the mobile base to the beginning of this path.

        :param allow_force_mode: Not used.
        """
        start_config = self._path_points[0]
        self._mobile.set_2d_pose(start_config[:3])
        self._path_done = False

    def set_to_end(self) -> None:
        """Sets the mobile base to the end of this path.

        :param allow_force_mode: Not used.
        """
        final_config = self._path_points[-1]
        self._mobile.set_2d_pose(final_config[:3])

    def visualize(self) -> None:
        """Draws a visualization of the path in the scene.

        The visualization can be removed
        with :py:meth:`ConfigurationPath.clear_visualization`.
        """
        if len(self._path_points) <= 0:
            raise RuntimeError("Can't visualise a path with no points.")

        tip = self._mobile
        self._drawing_handle = sim.simAddDrawingObject(
            objectType=sim.sim_drawing_lines, size=3, duplicateTolerance=0,
            parentObjectHandle=-1, maxItemCount=99999,
            ambient_diffuse=[1, 0, 1])
        sim.simAddDrawingObjectItem(self._drawing_handle, None)
        init_pose = self._mobile.get_2d_pose()
        self._mobile.set_2d_pose(self._path_points[0][:3])
        prev_point = list(tip.get_position())

        for i in range(len(self._path_points)):
            points = self._path_points[i]
            self._mobile.set_2d_pose(points[:3])
            p = list(tip.get_position())
            sim.simAddDrawingObjectItem(self._drawing_handle, prev_point + p)
            prev_point = p

        # Set the arm back to the initial config
        self._mobile.set_2d_pose(init_pose[:3])

    def clear_visualization(self) -> None:
        """Clears/removes a visualization of the path in the scene.
        """
        if self._drawing_handle is not None:
            sim.simAddDrawingObjectItem(self._drawing_handle, None)

    def _next_i_path(self):
        incr = 0.01
        dist_to_next = 0
        while dist_to_next < incr:
            self.i_path += 1
            if self.i_path == len(self._path_points) - 1:
                self.i_path = len(self._path_points) - 1
                break
            dist_to_next += self._path_points[self.i_path][-1]

    def _set_inter_target(self, i):
        self._mobile.intermediate_target_base.set_position(
            [self._path_points[i][0], self._path_points[i][1],
             self._mobile.target_z])
        self._mobile.intermediate_target_base.set_orientation(
            [0, 0, self._path_points[i][2]])
