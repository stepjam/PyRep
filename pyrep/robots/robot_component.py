from typing import List, Tuple

from pyrep.objects.shape import Shape

from pyrep.backend import sim, utils
from pyrep.const import JointType
from pyrep.objects.object import Object
from pyrep.objects.joint import Joint
from pyrep.const import ObjectType, JointMode


class RobotComponent(Object):
    """Collection of joints representing arms, end effectors, mobile bases, etc.
    """

    def __init__(self, count: int, name: str, joint_names: List[str],
                 base_name: str = None):
        suffix = '' if count == 0 else '#%d' % (count - 1)
        super().__init__(
            name + suffix if base_name is None else base_name + suffix)
        self._num_joints = len(joint_names)

        # Joint handles
        self.joints = [Joint(jname + suffix)
                       for jname in joint_names]
        self._joint_handles = [j.get_handle() for j in self.joints]

    def copy(self) -> 'RobotComponent':
        """Copy and pastes the arm in the scene.

        The arm is copied together with all its associated calculation
        objects and associated scripts.

        :return: The new pasted arm.
        """
        # Copy whole model
        handle = sim.simCopyPasteObjects([self._handle], 1)[0]
        name = sim.simGetObjectName(handle)
        # Find the number of this arm
        num = name[name.rfind('#') + 1:]
        if len(num) > 0:
            num = int(num) + 1
        else:
            num = 0
        # FIXME: Pass valid name and joint_names.
        return self.__class__(num)  # type: ignore

    def _get_requested_type(self) -> ObjectType:
        """Gets the type of the object.

        :return: Type of the object.
        """
        return ObjectType(sim.simGetObjectType(self.get_handle()))

    def get_joint_count(self) -> int:
        """Gets the number of joints in this component.

        :return: The number of joints.
        """
        return self._num_joints

    def get_joint_types(self) -> List[JointType]:
        """Retrieves the type of the joints in this component.

        :return: A list containing the types of the joints.
        """
        return [j.get_joint_type() for j in self.joints]

    def get_joint_positions(self) -> List[float]:
        """Retrieves the intrinsic position of the joints.

        See :py:meth:`Joint.get_joint_position` for more information.

        :return: A list of intrinsic position of the joints.
        """
        return [j.get_joint_position() for j in self.joints]

    def set_joint_positions(self, positions: List[float],
                            disable_dynamics: bool = False) -> None:
        """Sets the intrinsic position of the joints.

        See :py:meth:`Joint.set_joint_position` for more information.

        :param disable_dynamics: If True, then the position can be set even
            when the joint mode is in Force mode. It will disable dynamics,
            move the joint, and then re-enable dynamics.

        :param positions: A list of positions of the joints (angular or linear
            values depending on the joint type).
        """
        self._assert_len(positions)
        if not disable_dynamics:
            [sim.simSetJointPosition(jh, p)  # type: ignore
             for jh, p in zip(self._joint_handles, positions)]
            return

        is_model = self.is_model()
        if not is_model:
            self.set_model(True)

        prior = sim.simGetModelProperty(self.get_handle())
        p = prior | sim.sim_modelproperty_not_dynamic
        # Disable the dynamics
        sim.simSetModelProperty(self._handle, p)
        with utils.step_lock:
            sim.simExtStep(True)  # Have to step for changes to take effect

        [sim.simSetJointPosition(jh, p)  # type: ignore
         for jh, p in zip(self._joint_handles, positions)]
        [j.set_joint_target_position(p)  # type: ignore
         for j, p in zip(self.joints, positions)]
        with utils.step_lock:
            sim.simExtStep(True)  # Have to step for changes to take effect
        # Re-enable the dynamics
        sim.simSetModelProperty(self._handle, prior)
        self.set_model(is_model)

    def get_joint_target_positions(self) -> List[float]:
        """Retrieves the target positions of the joints.

        :return: A list of target position of the joints (angular or linear
            values depending on the joint type).
        """
        return [j.get_joint_target_position() for j in self.joints]

    def set_joint_target_positions(self, positions: List[float]) -> None:
        """Sets the target positions of the joints.

        See :py:meth:`Joint.set_joint_target_position` for more information.

        :param positions: List of target position of the joints (angular or
            linear values depending on the joint type).
        """
        self._assert_len(positions)
        [j.set_joint_target_position(p)  # type: ignore
         for j, p in zip(self.joints, positions)]

    def get_joint_target_velocities(self) -> List[float]:
        """Retrieves the intrinsic target velocities of the joints.

         :return: List of the target velocity of the joints (linear or angular
            velocity depending on the joint-type).
         """
        return [j.get_joint_target_velocity()  # type: ignore
                for j in self.joints]

    def set_joint_target_velocities(self, velocities: List[float]) -> None:
        """Sets the intrinsic target velocities of the joints.

        :param velocities: List of the target velocity of the joints (linear
            or angular velocities depending on the joint-type).
        """
        self._assert_len(velocities)
        [j.set_joint_target_velocity(v)  # type: ignore
         for j, v in zip(self.joints, velocities)]

    def get_joint_forces(self) -> List[float]:
        """Retrieves the forces or torques of the joints.

        See :py:meth:`Joint.get_joint_force` for more information.

        :return: A list of the forces or the torques applied to the joints
            along/about their z-axis.
        """
        return [j.get_joint_force() for j in self.joints]

    def set_joint_forces(self, forces: List[float]) -> None:
        """Sets the maximum force or torque that the joints can exert.

        See :py:meth:`Joint.set_joint_force` for more information.

        :param forces: The maximum force or torque that the joints can exert.
            These cannot be negative values.
        """
        self._assert_len(forces)
        [j.set_joint_force(f)  # type: ignore
         for j, f in zip(self.joints, forces)]

    def get_joint_velocities(self) -> List[float]:
        """Get the current joint velocities.

        :return: List containing the velocities of the joints (linear or
            angular velocities depending on the joint-type).
        """
        return [j.get_joint_velocity() for j in self.joints]

    def get_joint_intervals(self) -> Tuple[List[bool], List[List[float]]]:
        """Retrieves the interval parameters of the joints.

        See :py:meth:`Joint.get_joint_interval` for more information.

        :return: A tuple containing a list of bools indicates whether the joint
            is cyclic (the joint varies between -pi and +pi in a cyclic manner),
            and a 2D list containing the interval of the joints.
        """
        cyclics, intervals = [], []
        for j in self.joints:
            c, i = j.get_joint_interval()
            cyclics.append(c)
            intervals.append(i)
        return cyclics, intervals

    def set_joint_intervals(self, cyclic: List[bool],
                            intervals: List[List[float]]) -> None:
        """Sets the interval parameters of the joints (i.e. range values).

        See :py:meth:`Joint.set_joint_interval` for more information.

        :param cyclic: List of bools indicates whether the joint is cyclic.
            Only revolute joints with a pitch of 0 can be cyclic.
        :param intervals: 2D list containing the intervals of the joints.
        """
        self._assert_len(cyclic)
        self._assert_len(intervals)
        [j.set_joint_interval(c, i)  # type: ignore
         for j, c, i in zip( self.joints, cyclic, intervals)]

    def get_joint_upper_velocity_limits(self) -> List[float]:
        """Gets upper velocity limits of the joints.

         :return: List of the upper velocity limits.
         """
        return [j.get_joint_upper_velocity_limit() for j in self.joints]

    def set_control_loop_enabled(self, value: bool) -> None:
        """Sets whether the control loop is enable for all joints.

        :param value: The new value for the control loop state.
        """
        [j.set_control_loop_enabled(value)  # type: ignore
         for j in self.joints]

    def set_motor_locked_at_zero_velocity(self, value: bool) -> None:
        """Sets if motor is locked when target velocity is zero for all joints.

        When enabled in velocity mode and its target velocity is zero, then the
        joint is locked in place.

        :param value: If the motors should be locked at zero velocity.
        """
        [j.set_motor_locked_at_zero_velocity(value)  # type: ignore
         for j in self.joints]

    def set_joint_mode(self, value: JointMode) -> None:
        """Sets the operation mode of the joint group.

        :param value: The new joint mode value.
        """
        [j.set_joint_mode(value)  # type: ignore
         for j in self.joints]

    def get_joint_modes(self) -> List[JointMode]:
        """Gets the operation mode of the joint group.

        :return: A list of joint modes.
        """
        return [j.get_joint_mode() for j in self.joints]

    def get_visuals(self) -> List[Object]:
        """Gets a list of the visual elements of this component.

        Can be useful for methods such as domain randomization.
        Should ideally be overridden for each robot.

        :return: A list of visual shapes.
        """
        tree = self.get_objects_in_tree(ObjectType.SHAPE, exclude_base=False)
        return [obj for obj in tree if 'visual' in obj.get_name()]

    def _assert_len(self, inputs: list) -> None:
        if len(self.joints) != len(inputs):
            raise RuntimeError(
                'Tried to set values for %d joints, but joint group consists '
                'of %d joints.' % (len(inputs), len(self.joints)))
