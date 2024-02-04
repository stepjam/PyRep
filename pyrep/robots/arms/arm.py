from ctypes import CFUNCTYPE, c_int
from pyrep.backend.sim import SimBackend
from pyrep.backend import sim_const as simc
from pyrep.objects.object import Object
from pyrep.objects.dummy import Dummy
from pyrep.robots.configuration_paths.arm_configuration_path import ArmConfigurationPath
from pyrep.robots.robot_component import RobotComponent
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.errors import ConfigurationError, ConfigurationPathError, IKError
from pyrep.const import ConfigurationPathAlgorithms as Algos
from typing import List, Union
import numpy as np
from pyrep.backend.stack import callback


CALLBACK_NAME = "ccallback0"


@callback
def collision_check_callback(state, data):
    count, name, num_joints, base_name = data
    joint_names = ["%s_joint%d" % (name, i + 1) for i in range(num_joints)]
    arm = RobotComponent(
        count, name, joint_names, None if len(base_name) == 0 else base_name
    )
    arm.set_joint_positions(state)
    return not arm.check_collision()


class Arm(RobotComponent):
    """Base class representing a robot arm with path planning support."""

    def __init__(
        self,
        count: int,
        name: str,
        num_joints: int,
        base_name: str = None,
        max_velocity=1.0,
        max_acceleration=4.0,
        max_jerk=1000,
    ):
        """Count is used for when we have multiple copies of arms"""
        joint_names = ["%s_joint%d" % (name, i + 1) for i in range(num_joints)]
        super().__init__(count, name, joint_names, base_name)

        # Used for motion planning
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk

        # Motion planning handles
        suffix = "" if count == 0 else "#%d" % (count - 1)
        self._ik_target = Dummy("%s_target%s" % (name, suffix))
        self._ik_tip = Dummy("%s_tip%s" % (name, suffix))

        self._sim_ik_api = SimBackend().sim_ik_api
        self._sim_ompl_api = SimBackend().sim_ompl_api
        self._ik_env_handle = self._sim_ik_api.createEnvironment()
        self._ik_group_handle = self._sim_ik_api.createGroup(self._ik_env_handle)
        self.set_ik_group_properties("pseudo_inverse", 6, 0.02)

        base_handle = self.get_handle()
        target_handle = self._ik_target.get_handle()
        tip_handle = self._ik_tip.get_handle()

        (
            self._ik_element,
            self._sim_to_ik_map,
            self._ik_to_sim_map,
        ) = self._sim_ik_api.addElementFromScene(
            self._ik_env_handle,
            self._ik_group_handle,
            base_handle,
            tip_handle,
            target_handle,
            self._sim_ik_api.constraint_pose,
        )
        self._sim_ik_api.setElementPrecision(
            self._ik_env_handle,
            self._ik_group_handle,
            self._ik_element,
            [0.00005, 0.0017],
        )
        self._ik_joint_handles = [self._sim_to_ik_map[h] for h in self._joint_handles]
        self._ik_base_handle = self._sim_to_ik_map[base_handle]
        self._ik_target_handle = self._sim_to_ik_map[target_handle]
        self._ik_tip_handle = self._sim_to_ik_map[tip_handle]

        #  Not used, but needs to be kept in scope, or will be garbage collected
        self._collision_callback = CFUNCTYPE(c_int, c_int)(collision_check_callback)
        SimBackend().lib.simRegCallback(0, self._collision_callback)
        self._coll_callback_args = [
            count,
            name,
            num_joints,
            base_name if base_name is not None else "",
        ]

    def set_ik_element_properties(
        self,
        constraint_x=True,
        constraint_y=True,
        constraint_z=True,
        constraint_alpha_beta=True,
        constraint_gamma=True,
    ) -> None:
        constraints = 0
        if constraint_x:
            constraints |= self._sim_ik_api.constraint_x
        if constraint_y:
            constraints |= self._sim_ik_api.constraint_y
        if constraint_z:
            constraints |= self._sim_ik_api.constraint_z
        if constraint_alpha_beta:
            constraints |= self._sim_ik_api.constraint_alpha_beta
        if constraint_gamma:
            constraints |= self._sim_ik_api.constraint_gamma
        self._sim_ik_api.setElementConstraints(
            self._ik_env_handle, self._ik_group_handle, self._ik_element, constraints
        )

    def set_ik_group_properties(
        self, resolution_method="pseudo_inverse", max_iterations=6, dls_damping=0.1
    ) -> None:
        try:
            res_method = {
                "pseudo_inverse": self._sim_ik_api.method_pseudo_inverse,
                "damped_least_squares": self._sim_ik_api.method_damped_least_squares,
                "jacobian_transpose": self._sim_ik_api.method_jacobian_transpose,
            }[resolution_method]
        except KeyError:
            raise Exception(
                "Invalid resolution method. Must be one of "
                "['pseudo_inverse' | 'damped_least_squares' | 'jacobian_transpose']"
            )
        self._sim_ik_api.setGroupCalculation(
            self._ik_env_handle,
            self._ik_group_handle,
            res_method,
            dls_damping,
            max_iterations,
        )

    def solve_ik_via_sampling(
        self,
        position: Union[List[float], np.ndarray],
        euler: Union[List[float], np.ndarray] = None,
        quaternion: Union[List[float], np.ndarray] = None,
        ignore_collisions: bool = False,
        trials: int = 300,
        max_configs: int = 1,
        distance_threshold: float = 0.25,
        max_time_ms: float = 500,
        relative_to: Object = None,
    ) -> np.ndarray:
        """Solves an IK group and returns the calculated joint values.

        This IK method performs a random searches for manipulator configurations
        that matches the given end-effector pose in space. When the tip pose
        is close enough then IK is computed in order to try to bring the
        tip onto the target. This is the method that should be used when
        the start pose is far from the end pose.

        We generate 'max_configs' number of samples within X number of 'trials',
        before ranking them according to angular distance.

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :param ignore_collisions: If collision checking should be disabled.
        :param trials: The maximum number of attempts to reach max_configs.
        :param max_configs: The maximum number of configurations we want to
            generate before sorting them.
        :param distance_threshold: Distance indicating when IK should be
            computed in order to try to bring the tip onto the target.
        :param max_time_ms: Maximum time in ms spend searching for
            each configuation.
        :param relative_to: Indicates relative to which reference frame we want
            the target pose. Specify None to retrieve the absolute pose,
            or an Object relative to whose reference frame we want the pose.
        :raises: ConfigurationError if no joint configuration could be found.

        :return: 'max_configs' number of joint configurations, ranked according
            to angular distance.
        """
        if not ((euler is None) ^ (quaternion is None)):
            raise ConfigurationError(
                "Specify either euler or quaternion values, but not both."
            )

        prev_pose = self._ik_target.get_pose()
        self._ik_target.set_position(position, relative_to)
        if euler is not None:
            self._ik_target.set_orientation(euler, relative_to)
        elif quaternion is not None:
            self._ik_target.set_quaternion(quaternion, relative_to)

        validation_callback = None
        if not ignore_collisions:
            validation_callback = CALLBACK_NAME

        # TODO: Need to implement collision checking.
        metric = [1, 1, 1, 0.1]
        valid_joint_positions = []
        max_time_s = float(max_time_ms) * 0.001
        self._sim_ik_api.setObjectPose(
            self._ik_env_handle,
            self._ik_target_handle,
            self._ik_target.get_pose().tolist(),
            self._sim_ik_api.handle_world,
        )
        initial_config = self.get_joint_positions()
        for i in range(trials):
            config = self._sim_ik_api.findConfig(
                self._ik_env_handle,
                self._ik_group_handle,
                self._ik_joint_handles,
                distance_threshold,
                max_time_s,
                metric,
                validation_callback,
                self._coll_callback_args,
            )
            if not np.allclose(self.get_tip().get_position(), position, atol=0.01):
                raise ConfigurationError("Found a config, but tip was not on target.")
            if config is None:
                continue
            # TODO: Look to get alternative config
            # config = self._sim_ik_api.getAlternateConfigs(
            #   self._ik_env_handle, self._ik_joint_handles)
            if len(config) > 0:
                valid_joint_positions.append(config)
            if len(valid_joint_positions) >= max_configs:
                break

        self.set_joint_positions(initial_config)
        self._ik_target.set_pose(prev_pose)
        if len(valid_joint_positions) == 0:
            raise ConfigurationError(
                "Could not find a valid joint configuration for desired "
                "end effector pose."
            )

        if len(valid_joint_positions) > 1:
            current_config = np.array(self.get_joint_positions())
            # Sort based on angular distance
            valid_joint_positions.sort(key=lambda x: np.linalg.norm(current_config - x))

        return np.array(valid_joint_positions)

    def solve_ik_via_jacobian(
        self,
        position: Union[List[float], np.ndarray],
        euler: Union[List[float], np.ndarray] = None,
        quaternion: Union[List[float], np.ndarray] = None,
        relative_to: Object = None,
    ) -> np.ndarray:
        """Solves an IK group and returns the calculated joint values.

        This IK method performs a linearisation around the current robot
        configuration via the Jacobian. The linearisation is valid when the
        start and goal pose are not too far away, but after a certain point,
        linearisation will no longer be valid. In that case, the user is better
        off using 'solve_ik_via_sampling'.

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :param relative_to: Indicates relative to which reference frame we want
        the target pose. Specify None to retrieve the absolute pose,
        or an Object relative to whose reference frame we want the pose.
        :return: A list containing the calculated joint values.
        """
        self._ik_target.set_position(position, relative_to)
        if euler is not None:
            self._ik_target.set_orientation(euler, relative_to)
        elif quaternion is not None:
            self._ik_target.set_quaternion(quaternion, relative_to)

        options = {
            # if true, then calculation will be preceeded by
            # simIK.syncFromSim and followed by simIK.syncToSim
            "syncWorlds": False,
        }
        self._sim_ik_api.setObjectPose(
            self._ik_env_handle,
            self._ik_target_handle,
            self._ik_target.get_pose().tolist(),
            self._sim_ik_api.handle_world,
        )
        result, reason, precision = self._sim_ik_api.handleGroup(
            self._ik_env_handle, self._ik_group_handle, options
        )
        if reason == self._sim_ik_api.calc_notperformed:
            raise IKError("IK failed. Calculation not performed.")
        elif reason == self._sim_ik_api.calc_cannotinvert:
            raise IKError("IK failed. Could not invert Jacobian.")
        elif reason == self._sim_ik_api.calc_notwithintolerance:
            raise IKError("IK failed. Calculation not within tolerance.")
        elif reason == self._sim_ik_api.calc_stepstoobig:
            raise IKError("IK failed. Calculation step too big.")
        elif reason == self._sim_ik_api.calc_limithit:
            raise IKError("IK failed. Calculation limit hit.")
        if result != self._sim_ik_api.result_success:
            raise IKError("IK failed for unknown reason.")
        joint_values = np.array(
            [
                self._sim_ik_api.getJointPosition(self._ik_env_handle, jh)
                for jh in self._ik_joint_handles
            ]
        )
        return joint_values

    def get_path_from_cartesian_path(self, path: CartesianPath) -> ArmConfigurationPath:
        """Translate a path from cartesian space, to arm configuration space.

        Note: It must be possible to reach the start of the path via a linear
        path, otherwise an error will be raised.

        :param path: A :py:class:`CartesianPath` instance to be translated to
            a configuration-space path.
        :raises: ConfigurationPathError if no path could be created.

        :return: A path in the arm configuration space.
        """
        full_path = []
        initial_config = self.get_joint_positions()
        delta = 0.05
        for i in np.linspace(delta, 1, int(1 / delta)):
            pos, ori = path.get_pose_on_path(i)
            intermediate_path = self.get_linear_path(pos, ori, steps=5)
            full_path.extend(intermediate_path._path_points)
            intermediate_path.set_to_end()
        self.set_joint_positions(initial_config)
        return ArmConfigurationPath(self, full_path)

    def get_linear_path(
        self,
        position: Union[List[float], np.ndarray],
        euler: Union[List[float], np.ndarray] = None,
        quaternion: Union[List[float], np.ndarray] = None,
        steps=50,
        ignore_collisions=False,
        relative_to: Object = None,
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
        :param relative_to: Indicates relative to which reference frame we want
        the target pose. Specify None to retrieve the absolute pose,
        or an Object relative to whose reference frame we want the pose.
        :raises: ConfigurationPathError if no path could be created.

        :return: A linear path in the arm configuration space.
        """
        if not ((euler is None) ^ (quaternion is None)):
            raise ConfigurationPathError(
                "Specify either euler or quaternion values, but not both."
            )

        prev_pose = self._ik_target.get_pose()
        self._ik_target.set_position(position, relative_to)
        if euler is not None:
            self._ik_target.set_orientation(euler, relative_to)
        elif quaternion is not None:
            self._ik_target.set_quaternion(quaternion, relative_to)

        validation_callback = None
        if not ignore_collisions:
            validation_callback = CALLBACK_NAME

        self._sim_ik_api.setObjectPose(
            self._ik_env_handle,
            self._ik_target_handle,
            self._ik_target.get_pose().tolist(),
            self._sim_ik_api.handle_world,
        )
        ret_floats = self._sim_ik_api.generatePath(
            self._ik_env_handle,
            self._ik_group_handle,
            self._ik_joint_handles,
            self._ik_tip_handle,
            steps,
            validation_callback,
            self._coll_callback_args,
        )
        self._ik_target.set_pose(prev_pose)
        if len(ret_floats) == 0:
            raise ConfigurationPathError("Could not create path.")
        return ArmConfigurationPath(self, ret_floats)

    def get_nonlinear_path(
        self,
        position: Union[List[float], np.ndarray],
        euler: Union[List[float], np.ndarray] = None,
        quaternion: Union[List[float], np.ndarray] = None,
        ignore_collisions=False,
        trials=300,
        max_configs=1,
        distance_threshold: float = 0.25,
        max_time_ms: int = 500,
        trials_per_goal=1,
        algorithm=Algos.RRTConnect,
        relative_to: Object = None,
    ) -> ArmConfigurationPath:
        """Gets a non-linear (planned) configuration path given a target pose.

        A path is generated by finding several configs for a pose, and ranking
        them according to the distance in configuration space (smaller is
        better).

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :param ignore_collisions: If collision checking should be disabled.
        :param trials: The maximum number of attempts to reach max_configs.
            See 'solve_ik_via_sampling'.
        :param max_configs: The maximum number of configurations we want to
            generate before sorting them. See 'solve_ik_via_sampling'.
        :param distance_threshold: Distance indicating when IK should be
            computed in order to try to bring the tip onto the target.
            See 'solve_ik_via_sampling'.
        :param max_time_ms: Maximum time in ms spend searching for
            each configuation. See 'solve_ik_via_sampling'.
        :param trials_per_goal: The number of paths per config we want to trial.
        :param algorithm: The algorithm for path planning to use.
        :param relative_to: Indicates relative to which reference frame we want
        the target pose. Specify None to retrieve the absolute pose,
        or an Object relative to whose reference frame we want the pose.
        :raises: ConfigurationPathError if no path could be created.

        :return: A non-linear path in the arm configuration space.
        """
        try:
            configs = self.solve_ik_via_sampling(
                position,
                euler,
                quaternion,
                ignore_collisions,
                trials,
                max_configs,
                distance_threshold,
                max_time_ms,
                relative_to,
            )
        except ConfigurationError as e:
            raise ConfigurationPathError("Could not create path.") from e

        task_handle = self._sim_ompl_api.createTask("pyrep_task")
        self._sim_ompl_api.setVerboseLevel(task_handle, 0)
        algo = getattr(self._sim_ompl_api.Algorithm, algorithm.value)
        self._sim_ompl_api.setAlgorithm(task_handle, algo)

        ss_handles = [
            self._sim_ompl_api.createStateSpaceForJoint(
                f"joint_{jh}_state_space", jh, 1
            )
            for jh in self._joint_handles
        ]
        self._sim_ompl_api.setStateSpace(task_handle, ss_handles)

        use_for_projection = weights = np.ones_like(self._joint_handles).tolist()
        self._sim_ompl_api.setStateSpaceForJoints(
            task_handle, self._joint_handles, use_for_projection, weights
        )
        self._sim_ompl_api.setStartState(task_handle, self.get_joint_positions())
        self._sim_ompl_api.setGoalState(task_handle, configs[0].tolist())
        if not ignore_collisions:
            collision_pairs = [self._collision_collection_handle, simc.sim_handle_all]
            self._sim_ompl_api.setCollisionPairs(task_handle, collision_pairs)
        self._sim_ompl_api.setup(task_handle)

        max_time_s = float(max_time_ms) * 0.001
        solved, planned_path = self._sim_ompl_api.compute(
            task_handle, max_time_s, -1, 300
        )
        self._sim_ompl_api.destroyTask(task_handle)

        if not solved or len(planned_path) == 0:
            raise ConfigurationPathError("Could not create path.")
        return ArmConfigurationPath(self, planned_path)

    def get_path(
        self,
        position: Union[List[float], np.ndarray],
        euler: Union[List[float], np.ndarray] = None,
        quaternion: Union[List[float], np.ndarray] = None,
        ignore_collisions=False,
        trials=300,
        max_configs=1,
        distance_threshold: float = 0.65,
        max_time_ms: int = 500,
        trials_per_goal=1,
        algorithm=Algos.RRTConnect,
        relative_to: Object = None,
    ) -> ArmConfigurationPath:
        """Tries to get a linear path, failing that tries a non-linear path.

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y, z position of the target.
        :param euler: The x, y, z orientation of the target (in radians).
        :param quaternion: A list containing the quaternion (x,y,z,w).
        :param ignore_collisions: If collision checking should be disabled.
        :param trials: The maximum number of attempts to reach max_configs.
            See 'solve_ik_via_sampling'.
        :param max_configs: The maximum number of configurations we want to
            generate before sorting them. See 'solve_ik_via_sampling'.
        :param distance_threshold: Distance indicating when IK should be
            computed in order to try to bring the tip onto the target.
            See 'solve_ik_via_sampling'.
        :param max_time_ms: Maximum time in ms spend searching for
            each configuation. See 'solve_ik_via_sampling'.
        :param trials_per_goal: The number of paths per config we want to trial.
        :param algorithm: The algorithm for path planning to use.
        :param relative_to: Indicates relative to which reference frame we want
        the target pose. Specify None to retrieve the absolute pose,
        or an Object relative to whose reference frame we want the pose.

        :raises: ConfigurationPathError if neither a linear or non-linear path
            can be created.
        :return: A linear or non-linear path in the arm configuration space.
        """
        try:
            p = self.get_linear_path(
                position,
                euler,
                quaternion,
                ignore_collisions=ignore_collisions,
                relative_to=relative_to,
            )
            return p
        except ConfigurationPathError:
            pass  # Allowed. Try again, but with non-linear.

        # This time if an exception is thrown, we dont want to catch it.
        p = self.get_nonlinear_path(
            position,
            euler,
            quaternion,
            ignore_collisions,
            trials,
            max_configs,
            distance_threshold,
            max_time_ms,
            trials_per_goal,
            algorithm,
            relative_to,
        )
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
        jacobian, errorVector = self._sim_ik_api.computeGroupJacobian(
            self._ik_env_handle, self._ik_group_handle
        )
        jacobian = np.array(jacobian).reshape((len(self.joints), 6), order="F")
        return jacobian

    def check_arm_collision(self, obj: "Object" = None) -> bool:
        """Checks whether two entities are colliding.

        :param obj: The other collidable object to check collision against,
            or None to check against all collidable objects. Note that objects
            must be marked as collidable!
        :return: If the object is colliding.
        """
        return self.check_collision(obj)
