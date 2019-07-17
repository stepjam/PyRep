from pyrep.backend import vrep, utils
import numpy as np
from typing import List
from pyrep.const import PYREP_SCRIPT_TYPE
from math import sqrt, radians


class ConfigurationPath(object):
    """A path expressed in joint configuration space.

    Paths are retrieved from an :py:class:`Mobile`, and are associated with the
    mobile base that generated the path.

    This class is used for executing motion along a path via the
    _get_base_actuation function employing a proportional controller.
    """

    def __init__(self, mobile: 'Mobile', path_points: List[float]):
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
        """ Make a step along the trajectory.

        Step forward by calling _get_base_actuation to get the velocity needed to be applied at the wheels.

        NOTE: This does not step the physics engine. This is left to the user.

        :return: If the end of the trajectory has been reached.

        """
        if self._path_done:
            raise RuntimeError('This path has already been completed. '
                               'If you want to re-run, then call set_to_start.')

        pos_inter = self._mobile.intermediate_target_base.get_position(relative_to=self._mobile.base_ref)

        if self._mobile.type_ is "two_wheels":
            if len(self._path_points) > 2: # Non-linear path
                if self.inter_done:
                    self._next_i_path()
                    self._set_inter_target(self.i_path)
                    self.inter_done = False

                if sqrt((pos_inter[0])**2 + (pos_inter[1])**2) < 0.1:
                    self.inter_done = True
                    actuation, self._path_done = self._mobile._get_base_actuation()
                else:
                    actuation, self._path_done = self._mobile._get_base_actuation()

                self._mobile.set_base_angular_velocites(actuation)

                if self.i_path == len(self._path_points)-1:
                    self._path_done = True

            else:
                actuation, self._path_done = self._mobile._get_base_actuation()
                self._mobile.set_base_angular_velocites(actuation)

        elif self._mobile.type_ is "omnidirectional":
            if len(self._path_points) > 2: # Non-linear path
                if self.inter_done:
                    self._next_i_path()
                    self._set_inter_target(self.i_path)
                    self.inter_done = False

                    handleBase = self._mobile.base_ref.get_handle()
                    handleInterTargetBase = self._mobile.intermediate_target_base.get_handle()

                    __, ret_floats, _, _ = utils.script_call(
                        'getBoxAdjustedMatrixAndFacingAngle@PyRep', PYREP_SCRIPT_TYPE,
                        ints=[handleBase, handleInterTargetBase])

                    m = ret_floats[:-1]
                    angle = ret_floats[-1]
                    self._mobile.intermediate_target_base.set_position([m[3],m[7],self._mobile.target_z])
                    self._mobile.intermediate_target_base.set_orientation([0,0,angle])

                if sqrt((pos_inter[0])**2 + (pos_inter[1])**2) < 0.1:
                    self.inter_done = True
                    actuation = [0,0,0]
                else:
                    actuation, _ = self._mobile._get_base_actuation()

                self._mobile.set_base_angular_velocites(actuation)

                if self.i_path == len(self._path_points)-1:
                    self._path_done = True
            else:
                actuation, self._path_done = self._mobile._get_base_actuation()
                self._mobile.set_base_angular_velocites(actuation)

        return actuation, self._path_done

    def set_to_start(self) -> None:
        """Sets the mobile base to the beginning of this path.
        """
        start_config = self._path_points[0]
        self._mobile.set_base_position([start_config[0],start_config[1]])
        self._mobile.set_base_orientation(start_config[2])
        self._path_done = False

    def set_to_end(self) -> None:
        """Sets the mobile base to the end of this path.
        """
        final_config = self._path_points[-1]
        self._mobile.set_base_position([final_config[0],final_config[1]])
        self._mobile.set_base_orientation(final_config[2])

    def visualize(self) -> None:
        """Draws a visualization of the path in the scene.

        The visualization can be removed
        with :py:meth:`ConfigurationPath.clear_visualization`.
        """
        if len(self._path_points) <= 0:
            raise RuntimeError("Can't visualise a path with no points.")

        tip = self._mobile.get_tip()
        self._drawing_handle = vrep.simAddDrawingObject(
            objectType=vrep.sim_drawing_lines, size=3, duplicateTolerance=0,
            parentObjectHandle=-1, maxItemCount=99999,
            ambient_diffuse=[1, 0, 1])
        vrep.simAddDrawingObjectItem(self._drawing_handle, None)
        init_pos = self._mobile.get_base_position()
        init_angle = self._mobile.get_base_orientation()
        self._mobile.set_base_position(self._path_points[0][0:2])
        self._mobile.set_base_orientation(self._path_points[0][2])
        prev_point = tip.get_position()

        for i in range(len(self._path_points)):
            points = self._path_points[i]
            self._mobile.set_base_position(points[:2])
            self._mobile.set_base_orientation(points[2])
            p = tip.get_position()
            vrep.simAddDrawingObjectItem(self._drawing_handle, prev_point + p)
            prev_point = p

        # Set the arm back to the initial config
        self._mobile.set_base_position(init_pos)
        self._mobile.set_base_orientation(init_angle)


    def clear_visualization(self) -> None:
        """Clears/removes a visualization of the path in the scene.
        """
        if self._drawing_handle is not None:
            vrep.simAddDrawingObjectItem(self._drawing_handle, None)

    def _next_i_path(self):
        incr = 0.01
        dist_to_next = 0
        while dist_to_next < incr:
            self.i_path += 1
            if self.i_path ==  len(self._path_points) - 1:
                self.i_path = len(self._path_points) - 1
                break
            dist_to_next += self._path_points[self.i_path][-1]

    def _set_inter_target(self,i):
        self._mobile.intermediate_target_base.set_position([self._path_points[i][0],self._path_points[i][1],self._mobile.target_z])
        self._mobile.intermediate_target_base.set_orientation([0,0,self._path_points[i][2]])
