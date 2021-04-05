from pyrep.backend import sim, utils
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.robots.robot_component import RobotComponent
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.errors import ConfigurationPathError
from pyrep.const import PYREP_SCRIPT_TYPE
from typing import List, Union
from math import sqrt
import numpy as np


class MobileBase(RobotComponent):
    """Base class representing a robot mobile base with path planning support.
    """

    def __init__(self, count: int, num_wheels: int, name: str):
        """Count is used for when we have multiple copies of mobile bases.

        :param count: used for multiple copies of robots
        :param num_wheels: number of actuated wheels
        :param name: string with robot name (same as base in vrep model).
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

        self._collision_collection = sim.simGetCollectionHandle(
            '%s_base%s' % (name, suffix))

        # Robot parameters and handle
        self.z_pos = self.get_position()[2]
        self.target_z = self.target_base.get_position()[-1]
        self.wheel_radius = self.wheels[0].get_bounding_box()[5]  # Z
        self.wheel_distance = np.linalg.norm(
            np.array(self.wheels[0].get_position()) -
            np.array(self.wheels[1].get_position()))

        # Make sure dummies are orphan if loaded with ttm
        self.intermediate_target_base.set_parent(None)
        self.target_base.set_parent(None)

    def get_2d_pose(self) -> np.ndarray:
        """Gets the 2D (top-down) pose of the robot [x, y, yaw].

        :return: A List containing the x, y, yaw (in radians).
        """
        return np.r_[self.get_position()[:2], self.get_orientation()[-1:]]

    def set_2d_pose(self, pose: Union[List[float], np.ndarray]) -> None:
        """Sets the 2D (top-down) pose of the robot [x, y, yaw]

        :param pose: A List containing the x, y, yaw (in radians).
        """
        x, y, yaw = pose
        self.set_position([x, y, self.z_pos])
        self.set_orientation([0, 0, yaw])

    def assess_collision(self):
        """Silent detection of the robot base with all other entities present in the scene.

        :return: True if collision is detected
        """
        return sim.simCheckCollision(self._collision_collection,
                                     sim.sim_handle_all) == 1

    def set_cartesian_position(self, position: List[float]):
        """Set a delta target position (x,y) and rotation position

        :param position: length 3 list containing x and y position, and angle position

        NOTE: not supported for nonholonomic mobile bases.
        """
        vel = [0, 0, 0]
        vel[-1] = position[-1]
        for i in range(2):
            vel[i] = position[i] / (0.05 * self.wheel_radius / 2)  # "0.05 is dt"

        self.set_base_angular_velocites(vel)

    def set_base_angular_velocites(self, velocity: List[float]):
        """This function has no effect for two_wheels robot. More control is required for omnidirectional robot.

        :param velocity: for two wheels robot: each wheel velocity, for omnidirectional robot forwardBackward, leftRight and rotation velocity
        """
        raise NotImplementedError()

    def _get_nonlinear_path_points(self, position: List[float],
                           angle=0,
                           boundaries=2,
                           path_pts=600,
                           ignore_collisions=False,
                           algorithm=Algos.RRTConnect) -> List[List[float]]:
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

        # Base dummy required to be parent of the robot tree
        # self.base_ref.set_parent(None)
        # self.set_parent(self.base_ref)

        # Missing the dist1 for intermediate target

        self.target_base.set_position([position[0], position[1], self.target_z])
        self.target_base.set_orientation([0, 0, angle])

        handle_base = self.get_handle()
        handle_target_base = self.target_base.get_handle()

        _, ret_floats, _, _ = utils.script_call(
            'getNonlinearPathMobile@PyRep', PYREP_SCRIPT_TYPE,
            ints=[handle_base, handle_target_base,
                  self._collision_collection,
                  int(ignore_collisions), path_pts], floats=[boundaries],
                  strings=[algorithm.value])

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

    def _check_collision_linear_path(self,path):
        """Check for collision on a linear path from start to goal

        :param path: A list containing start and goal as [x,y,yaw]
        :return: A bool, True if collision was detected
        """
        start = path[0]
        end = path[1]

        m = (end[1] - start[1])/(end[0] - start[0])
        b = start[1] - m * start[0]
        x_range = [start[0],end[0]]
        x_span = start[0] - end[0]

        incr = round(abs(x_span)/50, 3)
        if x_range[1] < x_range[0]:
            incr = - incr

        x = x_range[0]
        for k in range(50):
            x += incr
            y = m * x + b
            self.set_2d_pose([x,y,start[-1] if k < 46 else end[-1]])
            status_collision = self.assess_collision()
            if status_collision == True:
                break

        return status_collision

    def get_base_actuation(self):
        """Controller for mobile bases.
        """
        raise NotImplementedError()

    def copy(self) -> RobotComponent:
        self.intermediate_target_base.set_parent(self)
        self.target_base.set_parent(self)
        c = super().copy()
        self.intermediate_target_base.set_parent(None)
        self.target_base.set_parent(None)
        return c
