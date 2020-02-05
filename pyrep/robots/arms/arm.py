from pyrep.backend import sim, utils
from pyrep.objects.dummy import Dummy
from pyrep.robots.configuration_paths.arm_configuration_path import (
    ArmConfigurationPath)
from pyrep.robots.robot_component import RobotComponent
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.errors import ConfigurationError, ConfigurationPathError, IKError
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.const import PYREP_SCRIPT_TYPE
from typing import List, Union
import numpy as np


class Arm(RobotComponent):
    """Base class representing a robot arm with path planning support.
    """

    def __init__(self, count: int, name: str, num_joints: int,
                 base_name: str = None,
                 max_velocity=1.0, max_acceleration=4.0, max_jerk=1000):
        """Count is used for when we have multiple copies of arms"""
        joint_names = ['%s_joint%d' % (name, i+1) for i in range(num_joints)]
        super().__init__(count, name, joint_names, base_name)

        # Used for motion planning
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk

        # Motion planning handles
        suffix = '' if count == 0 else '#%d' % (count - 1)
        self._ik_target = Dummy('%s_target%s' % (name, suffix))
        self._ik_tip = Dummy('%s_tip%s' % (name, suffix))
        self._ik_group = sim.simGetIkGroupHandle('%s_ik%s' % (name, suffix))
        self._collision_collection = sim.simGetCollectionHandle(
            '%s_arm%s' % (name, suffix))

    def get_configs_for_tip_pose(self,
                                 position: Union[List[float], np.ndarray],
                                 euler: Union[List[float], np.ndarray] = None,
                                 quaternion: Union[List[float], np.ndarray] = None,
                                 ignore_collisions=False,
                                 trials=300, max_configs=60) -> List[List[float]]:

        """Gets a valid joint configuration for a desired end effector pose.
        Must specify either rotation in euler or quaternions, but not both!
        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :param ignore_collisions: If collision checking should be disabled.
        :param trials: The maximum number of attempts to reach max_configs
        :param max_configs: The maximum number of configurations we want to
            generate before ranking them.
        :raises: ConfigurationError if no joint configuration could be found.
        :return: A list of valid joint configurations for the desired end effector pose.
        """

        if not ((euler is None) ^ (quaternion is None)):
            raise ConfigurationPathError(
                'Specify either euler or quaternion values, but not both.')

        prev_pose = self._ik_target.get_pose()
        self._ik_target.set_position(position)
        if euler is not None:
            self._ik_target.set_orientation(euler)
        elif quaternion is not None:
            self._ik_target.set_quaternion(quaternion)

        handles = [j.get_handle() for j in self.joints]

        # Despite verbosity being set to 0, OMPL spits out a lot of text
        with utils.suppress_std_out_and_err():
            _, ret_floats, _, _ = utils.script_call(
                'findSeveralCollisionFreeConfigsAndCheckApproach@PyRep', PYREP_SCRIPT_TYPE,
                ints=[self._ik_group, self._collision_collection,
                      int(ignore_collisions), trials, max_configs] + handles)
        self._ik_target.set_pose(prev_pose)

        if len(ret_floats) == 0:
            raise ConfigurationError(
                'Could not find a valid joint configuration for desired end effector pose.')

        num_configs = int(len(ret_floats)/len(handles))
        return [[ret_floats[len(handles)*i+j] for j in range(len(handles))] for i in range(num_configs)]

    def solve_ik(self, position: Union[List[float], np.ndarray],
                 euler: Union[List[float], np.ndarray] = None,
                 quaternion: Union[List[float], np.ndarray] = None) -> List[float]:
        """Solves an IK group and returns the calculated joint values.

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :return: A list containing the calculated joint values.
        """
        self._ik_target.set_position(position)
        if euler is not None:
            self._ik_target.set_orientation(euler)
        elif quaternion is not None:
            self._ik_target.set_quaternion(quaternion)

        ik_result, joint_values = sim.simCheckIkGroup(
            self._ik_group, [j.get_handle() for j in self.joints])
        if ik_result == sim.sim_ikresult_fail:
            raise IKError('IK failed. Perhaps the distance was between the tip '
                          ' and target was too large.')
        elif ik_result == sim.sim_ikresult_not_performed:
            raise IKError('IK not performed.')
        return joint_values

    def get_path_from_cartesian_path(self, path: CartesianPath
                                     ) -> ArmConfigurationPath:
        """Translate a path from cartesian space, to arm configuration space.

        Note: It must be possible to reach the start of the path via a linear
        path, otherwise an error will be raised.

        :param path: A :py:class:`CartesianPath` instance to be translated to
            a configuration-space path.
        :raises: ConfigurationPathError if no path could be created.

        :return: A path in the arm configuration space.
        """
        handles = [j.get_handle() for j in self.joints]
        _, ret_floats, _, _ = utils.script_call(
            'getPathFromCartesianPath@PyRep', PYREP_SCRIPT_TYPE,
            ints=[path.get_handle(), self._ik_group,
                  self._ik_target.get_handle()] + handles)
        if len(ret_floats) == 0:
            raise ConfigurationPathError(
                'Could not create a path from cartesian path.')
        return ArmConfigurationPath(self, ret_floats)

    def get_linear_path(self, position: Union[List[float], np.ndarray],
                        euler: Union[List[float], np.ndarray] = None,
                        quaternion: Union[List[float], np.ndarray] = None,
                        steps=50, ignore_collisions=False
                        ) -> ArmConfigurationPath:
        """Gets a linear configuration path given a target pose.

        Generates a path that drives a robot from its current configuration
        to its target dummy in a straight line (i.e. shortest path in Cartesian
        space).

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :param steps: The desired number of path points. Each path point
            contains a robot configuration. A minimum of two path points is
            required. If the target pose distance is large, a larger number
            of steps leads to better results for this function.
        :param ignore_collisions: If collision checking should be disabled.
        :raises: ConfigurationPathError if no path could be created.

        :return: A linear path in the arm configuration space.
        """
        if not ((euler is None) ^ (quaternion is None)):
            raise ConfigurationPathError(
                'Specify either euler or quaternion values, but not both.')

        prev_pose = self._ik_target.get_pose()
        self._ik_target.set_position(position)
        if euler is not None:
            self._ik_target.set_orientation(euler)
        elif quaternion is not None:
            self._ik_target.set_quaternion(quaternion)
        handles = [j.get_handle() for j in self.joints]

        # Despite verbosity being set to 0, OMPL spits out a lot of text
        with utils.suppress_std_out_and_err():
            _, ret_floats, _, _ = utils.script_call(
                'getLinearPath@PyRep', PYREP_SCRIPT_TYPE,
                ints=[steps, self._ik_group, self._collision_collection,
                      int(ignore_collisions)] + handles)
        self._ik_target.set_pose(prev_pose)

        if len(ret_floats) == 0:
            raise ConfigurationPathError('Could not create path.')
        return ArmConfigurationPath(self, ret_floats)

    def get_nonlinear_path(self, position: Union[List[float], np.ndarray],
                           euler: Union[List[float], np.ndarray] = None,
                           quaternion: Union[List[float], np.ndarray] = None,
                           ignore_collisions=False,
                           trials=100, max_configs=60, trials_per_goal=6,
                           algorithm=Algos.SBL) -> ArmConfigurationPath:
        """Gets a non-linear (planned) configuration path given a target pose.

        A path is generated by finding several configs for a pose, and ranking
        them according to the distance in configuration space (smaller is
        better).

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :param ignore_collisions: If collision checking should be disabled.
        :param trials: The maximum number of attempts to reach max_configs
        :param max_configs: The maximum number of configurations we want to
            generate before ranking them.
        :param trials_per_goal: The number of paths per config we want to trial.
        :param algorithm: The algorithm for path planning to use.
        :raises: ConfigurationPathError if no path could be created.

        :return: A non-linear path in the arm configuration space.
        """

        if not ((euler is None) ^ (quaternion is None)):
            raise ConfigurationPathError(
                'Specify either euler or quaternion values, but not both.')

        prev_pose = self._ik_target.get_pose()
        self._ik_target.set_position(position)
        if euler is not None:
            self._ik_target.set_orientation(euler)
        elif quaternion is not None:
            self._ik_target.set_quaternion(quaternion)

        handles = [j.get_handle() for j in self.joints]

        # Despite verbosity being set to 0, OMPL spits out a lot of text
        with utils.suppress_std_out_and_err():
            _, ret_floats, _, _ = utils.script_call(
                'getNonlinearPath@PyRep', PYREP_SCRIPT_TYPE,
                ints=[self._ik_group, self._collision_collection,
                      int(ignore_collisions), trials, max_configs,
                      trials_per_goal] + handles, strings=[algorithm.value])
        self._ik_target.set_pose(prev_pose)

        if len(ret_floats) == 0:
            raise ConfigurationPathError('Could not create path.')
        return ArmConfigurationPath(self, ret_floats)

    def get_path(self, position: Union[List[float], np.ndarray],
                 euler: Union[List[float], np.ndarray] = None,
                 quaternion: Union[List[float], np.ndarray] = None,
                 ignore_collisions=False,
                 trials=100, max_configs=60, trials_per_goal=6,
                 algorithm=Algos.SBL
                 ) -> ArmConfigurationPath:
        """Tries to get a linear path, failing that tries a non-linear path.

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :param ignore_collisions: If collision checking should be disabled.
        :param trials: The maximum number of attempts to reach max_configs.
            (Only applicable if a non-linear path is needed)
        :param max_configs: The maximum number of configurations we want to
            generate before ranking them.
            (Only applicable if a non-linear path is needed)
        :param trials_per_goal: The number of paths per config we want to trial.
            (Only applicable if a non-linear path is needed)
        :param algorithm: The algorithm for path planning to use.
            (Only applicable if a non-linear path is needed)

        :raises: ConfigurationPathError if neither a linear or non-linear path
            can be created.
        :return: A linear or non-linear path in the arm configuration space.
        """
        try:
            p = self.get_linear_path(position, euler, quaternion,
                                     ignore_collisions=ignore_collisions)
            return p
        except ConfigurationPathError:
            pass  # Allowed. Try again, but with non-linear.

        # This time if an exception is thrown, we dont want to catch it.
        p = self.get_nonlinear_path(
            position, euler, quaternion, ignore_collisions, trials, max_configs,
            trials_per_goal, algorithm)
        return p

    def get_tip(self) -> Dummy:
        """Gets the tip of the arm.

        Each arm is required to have a tip for path planning.

        :return: The tip of the arm.
        """
        return self._ik_tip

    def get_jacobian(self):
        """Calculates the Jacobian.

        :return: the row-major Jacobian matix.
        """
        self._ik_target.set_matrix(self._ik_tip.get_matrix())
        sim.simCheckIkGroup(self._ik_group,
                            [j.get_handle() for j in self.joints])
        jacobian, (rows, cols) = sim.simGetIkGroupMatrix(self._ik_group, 0)
        jacobian = np.array(jacobian).reshape((rows, cols), order='F')
        return jacobian
