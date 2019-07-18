from pyrep.backend import vrep, utils
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.robots.robot_component import RobotComponent
from pyrep.robots.configuration_paths.mobile_configuration_path import (
    MobileConfigurationPath)
from pyrep.errors import ConfigurationPathError
from pyrep.const import PYREP_SCRIPT_TYPE
from contextlib import contextmanager
from typing import List
import sys
import os
import io
from math import pi, sqrt


class MobileBase(RobotComponent):
    """Base class representing a robot mobile base with path planning support.
    """

    def __init__(self,
                 count: int,
                 num_wheels: int,
                 distance_from_target: float,
                 name: str,
                 max_velocity: float = 4,
                 max_velocity_rotation: float = 6,
                 max_acceleration: float = 0.035):
        """Count is used for when we have multiple copies of mobile bases.

        max_velocity, max_velocityRot, max_acceleration are not used for now for two_wheels robot.
        distance_from_target is implemented for omnidirectional robot only. It will solve the task
        by reaching at a distance (distance_from_target) from the target.
        """
        joint_names = ['%s_m_joint%s' % (name, str(i + 1)) for i in
                       range(num_wheels)]
        super().__init__(count, name, joint_names)

        self.num_wheels = num_wheels
        suffix = '' if count == 0 else '#%d' % (count - 1)

        wheel_names = ['%s_wheel%s%s' % (name, str(i + 1), suffix) for i in
                       range(self.num_wheels)]
        self.wheels = [Shape(name) for name in wheel_names]

        # Motion planning handles
        self.intermediate_target_base = Dummy(
            '%s_intermediate_target_base%s' % (name, suffix))
        self.target_base = Dummy('%s_target_base%s' % (name, suffix))

        self._collision_collection = vrep.simGetCollectionHandle(
            '%s_base%s' % (name, suffix))

        # Robot parameters and handle
        self.z_pos = self.get_position()[2]
        self.target_z = self.target_base.get_position()[-1]
        self.wheel_size = self.wheels[0].get_bounding_box()[1] * 2
        self.wheel_sep = abs(
            self.wheels[0].get_position()[1] - self.wheels[1].get_position()[
                1]) / 2

        # Make sure dummies are orphan if loaded with ttm
        self.intermediate_target_base.set_parent(None)
        self.target_base.set_parent(None)

    def get_2d_pose(self) -> List[float]:
        """Gets the 2D (top-down) pose of the robot [x, y, yaw].

        :return: A List containing the x, y, yaw (in radians).
        """
        return (self.get_position()[:2] +
                self.get_orientation()[-1:])

    def set_2d_pose(self, pose: List[float]) -> None:
        """Sets the 2D (top-down) pose of the robot [x, y, yaw]

        :param pose: A List containing the x, y, yaw (in radians).
        """
        x, y, yaw = pose
        self.set_position([x, y, self.z_pos])
        self.set_orientation([0, 0, yaw])

    def assess_collision(self):
        """ Silent detection of the robot base with all other entities present in the scene.

        :return: True if collision is detected
        """
        return vrep.simCheckCollision(self._collision_collection,
                                      vrep.sim_handle_all) == 1

    def set_cartesian_position(self, position: List[float]):
        """ Set a delta target position (x,y) and rotation position

        :param position: length 3 list containing x and y position, and angle position

        NOTE: not supported for two wheel robot yet.
        """
        vel = [0, 0, 0]
        vel[-1] = position[-1]
        for i in range(2):
            vel[i] = position[i] / (0.05 * self.wheel_size / 2)  # "0.05 is dt"

        self.set_base_angular_velocites(vel)

    def set_base_angular_velocites(self, velocity: List[float]):
        """ This function has no effect for two_wheels robot. More control is required for omnidirectional robot.

        :param velocity: for two wheels robot: each wheel velocity, for omnidirectional robot forwardBackward, leftRight and rotation velocity
        """
        raise NotImplementedError()

    def _get_nonlinear_path_points(self, position: List[float],
                           angle=0,
                           boundaries=2,
                           path_pts=600,
                           ignore_collisions=False) -> List[List[float]]:
        """Gets a non-linear (planned) configuration path given a target pose.

        :param position: The x, y, z position of the target.
        :param angle: The z orientation of the target (in radians).
        :param boundaries: A float defining the path search in x and y direction
        [[-boundaries,boundaries],[-boundaries,boundaries]].
        :param path_pts: number of sampled points returned from the computed path
        :param ignore_collisions: If collision checking should be disabled.

        :return: A non-linear path (x,y,angle) in the xy configuration space.
        """

        # Base dummy required to be parent of the robot tree
        # self.base_ref.set_parent(None)
        # self.set_parent(self.base_ref)

        # Missing the dist1 for intermediate target

        self.target_base.set_position([position[0], position[1], self.target_z])
        self.target_base.set_orientation([0, 0, angle])

        handle_base = self.get_handle()
        handle_target_base = self.target_base.get_handle()

        # Despite verbosity being set to 0, OMPL spits out a lot of text
        with suppress_std_out_and_err():
            _, ret_floats, _, _ = utils.script_call(
                'getNonlinearPathMobile@PyRep', PYREP_SCRIPT_TYPE,
                ints=[handle_base, handle_target_base,
                      self._collision_collection,
                      int(ignore_collisions), path_pts], floats=[boundaries])

        # self.set_parent(None)
        # self.base_ref.set_parent(self)

        if len(ret_floats) == 0:
            raise ConfigurationPathError('Could not create path.')

        path = []
        for i in range(0, len(ret_floats) // 3):
            inst = ret_floats[3 * i:3 * i + 3]
            if i > 0:
                dist_change = sqrt((inst[0] - prev_inst[0]) ** 2 + (
                inst[1] - prev_inst[1]) ** 2)
            else:
                dist_change = 0
            inst.append(dist_change)

            path.append(inst)

            prev_inst = inst

        return path

    def get_base_actuation(self):
        """ Controller for two wheels and omnidirectional robots.
        Based on a proportional controller. Used for motion planning.
        """
        raise NotImplementedError()

    def copy(self) -> RobotComponent:
        self.intermediate_target_base.set_parent(self)
        self.target_base.set_parent(self)
        c = super().copy()
        self.intermediate_target_base.set_parent(None)
        self.target_base.set_parent(None)
        return c


@contextmanager
def suppress_std_out_and_err():
    """Used for suppressing std out/err.

    This is needed because the OMPL plugin outputs logging info even when
    logging is turned off.
    """

    try:
        # If we are using an IDE, then this will fail
        original_stdout_fd = sys.stdout.fileno()
        original_stderr_fd = sys.stderr.fileno()
    except io.UnsupportedOperation:
        # Nothing we can do about this, just don't suppress
        yield
        return

    with open(os.devnull, "w") as devnull:

        devnull_fd = devnull.fileno()

        def _redirect_stdout(to_fd):
            sys.stdout.close()
            os.dup2(to_fd, original_stdout_fd)
            sys.stdout = io.TextIOWrapper(os.fdopen(original_stdout_fd, 'wb'))

        def _redirect_stderr(to_fd):
            sys.stderr.close()
            os.dup2(to_fd, original_stderr_fd)
            sys.stderr = io.TextIOWrapper(os.fdopen(original_stderr_fd, 'wb'))

        saved_stdout_fd = os.dup(original_stdout_fd)
        # saved_stderr_fd = os.dup(original_stderr_fd)

        try:
            _redirect_stdout(devnull_fd)
            # _redirect_stderr(devnull_fd)
            yield
            _redirect_stdout(saved_stdout_fd)
            # _redirect_stderr(saved_stderr_fd)
        finally:
            os.close(saved_stdout_fd)
            # os.close(saved_stderr_fd)
