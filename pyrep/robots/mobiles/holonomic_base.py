from pyrep.robots.mobiles.mobile_base import MobileBase
from pyrep.robots.configuration_paths.holonomic_configuration_path import (
    HolonomicConfigurationPath)
from pyrep.backend import utils
from pyrep.const import PYREP_SCRIPT_TYPE
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.errors import ConfigurationPathError
from typing import List
from pyrep.objects.joint import Joint
from math import pi, sqrt


class HolonomicBase(MobileBase):

    def __init__(self,
                 count: int,
                 num_wheels: int,
                 distance_from_target: float,
                 name: str,
                 max_velocity: float = 4,
                 max_velocity_rotation: float = 6,
                 max_acceleration: float = 0.035):
        """Init.

        :param count: used for multiple copies of robots.
        :param num_wheels: number of actuated wheels.
        :param distance_from_target: offset from target.
        :param name: string with robot name (same as base in vrep model).
        :param max_velocity: bounds x,y velocity for motion planning.
        :param max_velocity_rotation: bounds yaw velocity for motion planning.
        :param max_acceleration: bounds acceleration for motion planning.
        """

        super().__init__(count, num_wheels, name)

        suffix = '' if count == 0 else '#%d' % (count - 1)

        self.paramP = 20
        self.paramO = 10
        # self.paramO = 30
        self.previous_forw_back_vel = 0
        self.previous_left_right_vel = 0
        self.previous_rot_vel = 0
        self.accelF = max_acceleration
        self.maxV = max_velocity
        self.max_v_rot = max_velocity_rotation
        self.dist1 = distance_from_target

        joint_slipping_names = [
            '%s_slipping_m_joint%s%s' % (name, str(i + 1), suffix) for i in
            range(self.num_wheels)]
        self.joints_slipping = [Joint(jsname)
                                for jsname in joint_slipping_names]

    def set_base_angular_velocites(self, velocity: List[float]):
        """Calls required functions to achieve desired omnidirectional effect.

        :param velocity: A List with forwardBackward, leftRight and rotation
            velocity (in radian/s)
        """
        # self._reset_wheel()
        fBVel = velocity[0]
        lRVel = velocity[1]
        rVel = velocity[2]
        self.set_joint_target_velocities(
            [-fBVel - lRVel - rVel, -fBVel + lRVel - rVel,
             -fBVel - lRVel + rVel, -fBVel + lRVel + rVel])

    def get_linear_path(self, position: List[float],
                        angle=0) -> HolonomicConfigurationPath:
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

        handle_base = self.get_handle()
        handle_target_base = self.target_base.get_handle()
        _, ret_floats, _, _ = utils.script_call(
            'getBoxAdjustedMatrixAndFacingAngle@PyRep', PYREP_SCRIPT_TYPE,
            ints=[handle_base, handle_target_base])

        m = ret_floats[:-1]
        angle = ret_floats[-1]
        self.intermediate_target_base.set_position(
            [m[3] - m[0] * self.dist1, m[7] - m[4] * self.dist1,
             self.target_z])
        self.intermediate_target_base.set_orientation([0, 0, angle])
        self.target_base.set_orientation([0, 0, angle])

        path = [[position_base[0], position_base[1], angle_base],
                [position[0], position[1], angle]]

        if self._check_collision_linear_path(path):
            raise ConfigurationPathError(
                'Could not create path. '
                'An object was detected on the linear path.')

        return HolonomicConfigurationPath(self, path)

    def get_nonlinear_path(self, position: List[float],
                           angle=0,
                           boundaries=2,
                           path_pts=600,
                           ignore_collisions=False,
                           algorithm=Algos.RRTConnect
                            ) -> HolonomicConfigurationPath:
        """Gets a non-linear (planned) configuration path given a target pose.

        :param position: The x, y, z position of the target.
        :param angle: The z orientation of the target (in radians).
        :param boundaries: A float defining the path search in x and y direction
            [[-boundaries,boundaries],[-boundaries,boundaries]].
        :param path_pts: The number of sampled points returned from the
            computed path
        :param ignore_collisions: If collision checking should be disabled.
        :param algorithm: Algorithm used to compute path
        :raises: ConfigurationPathError if no path could be created.

        :return: A non-linear path (x,y,angle) in the xy configuration space.
        """

        path = self._get_nonlinear_path_points(
            position, angle, boundaries, path_pts, ignore_collisions, algorithm)

        return HolonomicConfigurationPath(self, path)

    def get_base_actuation(self):
        """Proportional controller.

        :return: A list with left and right joint velocity,
            and a bool representing target is reached.
        """

        handleBase = self.get_handle()
        handle_inter_target_base = self.intermediate_target_base.get_handle()
        pos_v = self.target_base.get_position(relative_to=self)
        or_v = self.target_base.get_orientation(relative_to=self)

        pos_inter = self.intermediate_target_base.get_position(
            relative_to=self)
        or_inter = self.intermediate_target_base.get_orientation(
            relative_to=self)

        if (sqrt((pos_v[0]) ** 2 + (
        pos_v[1]) ** 2) - self.dist1) < 0.001 and or_v[-1] < 0.1 * pi / 180:
            return [self.previous_forw_back_vel, self.previous_left_right_vel,
                    self.previous_rot_vel], True

        forw_back_vel = pos_inter[1] * self.paramP
        left_right_vel = pos_inter[0] * self.paramP
        rot_vel = - or_inter[2] * self.paramO

        v = sqrt(forw_back_vel * forw_back_vel +
                 left_right_vel * left_right_vel)
        if v > self.maxV:
            forw_back_vel = forw_back_vel * self.maxV / v
            left_right_vel = left_right_vel * self.maxV / v

        if (abs(rot_vel) > self.max_v_rot):
            rot_vel = self.max_v_rot * rot_vel / abs(rot_vel)

        df = forw_back_vel - self.previous_forw_back_vel
        ds = left_right_vel - self.previous_left_right_vel
        dr = rot_vel - self.previous_rot_vel

        if abs(df) > self.maxV * self.accelF:
            df = abs(df) * (self.maxV * self.accelF) / df

        if abs(ds) > self.maxV * self.accelF:
            ds = abs(ds) * (self.maxV * self.accelF) / ds

        if abs(dr) > self.max_v_rot * self.accelF:
            dr = abs(dr) * (self.max_v_rot * self.accelF) / dr

        forw_back_vel = self.previous_forw_back_vel + df
        left_right_vel = self.previous_left_right_vel + ds
        rot_vel = self.previous_rot_vel + dr

        self.previous_forw_back_vel = forw_back_vel
        self.previous_left_right_vel = left_right_vel
        self.previous_rot_vel = rot_vel

        return [forw_back_vel, left_right_vel, rot_vel], False

    def _reset_wheel(self):
        """Required to achieve desired omnidirectional wheel effect.
        """
        [j.reset_dynamic_object() for j in self.wheels]

        p = [[-pi / 4, 0, 0], [pi / 4, 0, pi], [-pi / 4, 0, 0], [pi / 4, 0, pi]]

        for i in range(self.num_wheels):
            self.joints_slipping[i].set_position([0, 0, 0],
                                                 relative_to=self.joints[i],
                                                 reset_dynamics=False)
            self.joints_slipping[i].set_orientation(p[i],
                                                    relative_to=self.joints[i],
                                                    reset_dynamics=False)
            self.wheels[i].set_position([0, 0, 0], relative_to=self.joints[i],
                                        reset_dynamics=False)
            self.wheels[i].set_orientation([0, 0, 0],
                                           relative_to=self.joints[i],
                                           reset_dynamics=False)
