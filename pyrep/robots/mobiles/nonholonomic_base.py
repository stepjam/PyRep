from pyrep.robots.mobiles.mobile_base import MobileBase
from pyrep.robots.configuration_paths.nonholonomic_configuration_path import (
    NonHolonomicConfigurationPath)
from pyrep.backend import utils
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.const import PYREP_SCRIPT_TYPE
from pyrep.errors import ConfigurationPathError
from typing import List
from math import sqrt


class NonHolonomicBase(MobileBase):

    def __init__(self,
                 count: int,
                 num_wheels: int,
                 distance_from_target: float,
                 name: str,
                 max_velocity: float = 4,
                 max_velocity_rotation: float = 6,
                 max_acceleration: float = 0.035):

        super().__init__(
            count, num_wheels, distance_from_target, name,
            max_velocity, max_velocity_rotation, max_acceleration)
        self.paramP = 0.1
        self.paramO = 0.8
        self.previous_forw_back_vel = 0
        self.previous_rot_vel = 0
        self.max_acceleration = max_acceleration
        self.max_velocity = max_velocity
        self.max_vertical_rotation = max_velocity_rotation
        self.distance_from_target = distance_from_target

    def get_linear_path(self, position: List[float],
                        angle=0) -> NonHolonomicConfigurationPath:
        """Initialize linear path and check for collision along it.

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y position of the target.
        :param angle: The z orientation of the target (in radians).
        :raises: ConfigurationPathError if no path could be created.

        :return: A linear path in the 2d space.
        """
        position_base = self.get_position()
        angle_base = self.get_orientation()[-1]

        self.target_base.set_position(
            [position[0], position[1], self.target_z])
        self.target_base.set_orientation([0, 0, angle])
        self.intermediate_target_base.set_position(
            [position[0], position[1], self.target_z])
        self.intermediate_target_base.set_orientation([0, 0, angle])

        path = [[position_base[0], position_base[1], angle_base],
                [position[0], position[1], angle]]

        if self._check_collision_linear_path(path):
            raise ConfigurationPathError('Could not create path. An object was detected on the linear path.')

        return NonHolonomicConfigurationPath(self, path)

    def get_nonlinear_path(self, position: List[float],
                           angle=0,
                           boundaries=2,
                           path_pts=600,
                           ignore_collisions=False,
                           algorithm=Algos.RRTConnect
                           ) -> NonHolonomicConfigurationPath:
        """Gets a non-linear (planned) configuration path given a target pose.

        :param position: The x, y, z position of the target.
        :param angle: The z orientation of the target (in radians).
        :param boundaries: A float defining the path search in x and y direction
        [[-boundaries,boundaries],[-boundaries,boundaries]].
        :param path_pts: number of sampled points returned from the computed path
        :param ignore_collisions: If collision checking should be disabled.
        :param algorithm: Algorithm used to compute path
        :raises: ConfigurationPathError if no path could be created.

        :return: A non-linear path (x,y,angle) in the xy configuration space.
        """

        path = self._get_nonlinear_path_points(
            position, angle, boundaries, path_pts, ignore_collisions)

        return NonHolonomicConfigurationPath(self, path)

    def get_base_actuation(self):
        """Proportional controller.

        :return: A list with left and right joint velocity, and bool if target is reached.
        """

        handleBase = self.get_handle()
        handle_inter_target_base = self.intermediate_target_base.get_handle()
        pos_v = self.target_base.get_position(relative_to=self)
        or_v = self.target_base.get_orientation(relative_to=self)

        pos_inter = self.intermediate_target_base.get_position(
            relative_to=self)
        __, ret_floats, _, _ = utils.script_call(
            'getAngleTwoWheel@PyRep', PYREP_SCRIPT_TYPE,
            ints=[handleBase, handle_inter_target_base])

        if sqrt((pos_v[0]) ** 2 + (pos_v[1]) ** 2) < 0.01:
            return [0, 0], True

        dist_vec = sqrt(pos_inter[0] ** 2 + pos_inter[1] ** 2)
        v_des = self.paramP * dist_vec
        omega_des = self.paramO * ret_floats[0]

        v_R = v_des + self.wheel_sep * omega_des
        v_L = v_des - self.wheel_sep * omega_des

        omega_jointR = v_R / (self.wheel_size / 2)
        omega_jointL = v_L / (self.wheel_size / 2)

        return [omega_jointL, omega_jointR], False
