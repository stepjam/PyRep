from math import atan2
from math import cos
from math import sin
from math import sqrt
from typing import List

from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.errors import ConfigurationPathError
from pyrep.robots.configuration_paths.nonholonomic_configuration_path import (
    NonHolonomicConfigurationPath)
from pyrep.robots.mobiles.mobile_base import MobileBase


class NonHolonomicBase(MobileBase):
    """Currently only differential drive robots.
    Can be refactored to include other types of non-holonomic bases in future.
    """

    def __init__(self,
                 count: int,
                 num_wheels: int,
                 name: str):

        super().__init__(count, num_wheels, name)

        self.cummulative_error = 0
        self.prev_error = 0

        # PID controller values.
        # TODO: expose to user through constructor.
        self.Kp = 1.0
        self.Ki = 0.01
        self.Kd = 0.1
        self.desired_velocity = 0.05

    def get_base_velocities(self) -> List[float]:
        """Gets linear and angular velocities of the mobile robot base
        calculated from kinematics equations. Left joint should have index 1,
        right joint should have index.

        :return: A list with linear and angular velocity of the robot base.
        """
        wv = self.get_joint_velocities()

        lv = sum(wv) * self.wheel_radius / 2.0
        v_diff = wv[1] - wv[0]
        av = self.wheel_radius * v_diff / self.wheel_distance

        return [lv, av]

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
            raise ConfigurationPathError(
                'Could not create path. '
                'An object was detected on the linear path.')

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
            position, angle, boundaries, path_pts, ignore_collisions, algorithm)

        return NonHolonomicConfigurationPath(self, path)

    def get_base_actuation(self):
        """A controller using PID.

        :return: A list with left and right joint velocity, and bool if target is reached.
        """

        d_x, d_y, _ = self.intermediate_target_base.get_position(
            relative_to=self)

        d_x_final, d_y_final, _ = self.target_base.get_position(
            relative_to=self)

        if sqrt((d_x_final) ** 2 + (d_y_final) ** 2) < 0.1:
            return [0., 0.], True

        alpha = atan2(d_y, d_x)
        e = atan2(sin(alpha), cos(alpha))
        e_P = e
        e_I = self.cummulative_error + e
        e_D = e - self.prev_error
        w = self.Kp * e_P + self.Ki * e_I + self.Kd * e_D
        w = atan2(sin(w), cos(w))

        self.cummulative_error = self.cummulative_error + e
        self.prev_error = e

        vr = ((2. * self.desired_velocity + w * self.wheel_distance) /
              (2. * self.wheel_radius))
        vl = ((2. * self.desired_velocity - w * self.wheel_distance) /
              (2. * self.wheel_radius))

        return [vl, vr], False
