from typing import Tuple, List, Union
from pyrep.backend import sim, utils
from pyrep.const import JointType, JointMode
from pyrep.objects.object import Object, object_type_to_class
from pyrep.const import ObjectType


class Joint(Object):
    """A joint or actuator.

    Four types are supported: revolute joints, prismatic joints,
    screws and spherical joints.
    """

    def __init__(self, name_or_handle: Union[str, int]):
        super().__init__(name_or_handle)

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.JOINT

    def get_joint_type(self) -> JointType:
        """Retrieves the type of a joint.

        :return: The type of the joint.
        """
        return JointType(sim.simGetJointType(self._handle))

    def get_joint_position(self) -> float:
        """Retrieves the intrinsic position of a joint.

        This function cannot be used with spherical joints.

        :return: Intrinsic position of the joint. This is a one-dimensional
            value: if the joint is revolute, the rotation angle is returned,
            if the joint is prismatic, the translation amount is returned, etc.
        """
        return sim.simGetJointPosition(self._handle)

    def set_joint_position(self, position: float,
                           disable_dynamics: bool = False) -> None:
        """Sets the intrinsic position of the joint.

        :param disable_dynamics: If True, then the position can be set even
            when the joint mode is in Force mode. It will disable dynamics,
            move the joint, and then re-enable dynamics.

        :param position: Position of a joint (angular or linear
            values depending on the joint type).
        """
        if not disable_dynamics:
            sim.simSetJointPosition(self._handle, position)
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

        sim.simSetJointPosition(self._handle, position)
        self.set_joint_target_position(position)

        with utils.step_lock:
            sim.simExtStep(True)  # Have to step for changes to take effect
        # Re-enable the dynamics
        sim.simSetModelProperty(self._handle, prior)
        self.set_model(is_model)

    def get_joint_target_position(self) -> float:
        """Retrieves the target position of a joint.

        :return: Target position of the joint (angular or linear value
            depending on the joint type).
        """
        return sim.simGetJointTargetPosition(self._handle)

    def set_joint_target_position(self, position: float) -> None:
        """Sets the target position of a joint.

        This command makes only sense when the joint is in torque/force mode
        (also make sure that the joint's motor and position control
        are enabled).

        :param position: Target position of the joint (angular or linear
            value depending on the joint type).
        """
        sim.simSetJointTargetPosition(self._handle, position)

    def get_joint_target_velocity(self) -> float:
        """Retrieves the intrinsic target velocity of a non-spherical joint.

        :return: Target velocity of the joint (linear or angular velocity
            depending on the joint-type).
        """
        return sim.simGetJointTargetVelocity(self._handle)

    def set_joint_target_velocity(self, velocity: float) -> None:
        """Sets the intrinsic target velocity of a non-spherical joint.

        This command makes only sense when the joint mode is torque/force
        mode: the dynamics functionality and the joint motor have to be
        enabled (position control should however be disabled).

        :param velocity: Target velocity of the joint (linear or angular
            velocity depending on the joint-type).
        """
        sim.simSetJointTargetVelocity(self._handle, velocity)

    def get_joint_force(self) -> float:
        """Retrieves the force or torque applied along/about its active axis.

        This function retrieves meaningful information only if the joint is
        prismatic or revolute, and is dynamically enabled. With the Bullet
        engine, this function returns the force or torque applied to the joint
        motor (torques from joint limits are not taken into account). With the
        ODE and Vortex engine, this function returns the total force or torque
        applied to a joint along/about its z-axis.

        :return: The force or the torque applied to the joint along/about
            its z-axis.
        """
        return sim.simGetJointForce(self._handle)

    def set_joint_force(self, force: float) -> None:
        """Sets the maximum force or torque that a joint can exert.

        The joint will apply that force/torque until the joint target velocity
        has been reached. To apply a negative force/torque, set a negative
        target velocity. This function has no effect when the joint is not
        dynamically enabled, or when it is a spherical joint.

        :param force: The maximum force or torque that the joint can exert.
            This cannot be a negative value.
        """
        sim.simSetJointMaxForce(self._handle, force)

    def get_joint_velocity(self) -> float:
        """Get the current joint velocity.

        :return: Velocity of the joint (linear or angular velocity depending
            on the joint-type).
        """
        return sim.simGetObjectFloatParameter(
            self._handle, sim.sim_jointfloatparam_velocity)

    def get_joint_interval(self) -> Tuple[bool, List[float]]:
        """Retrieves the interval parameters of a joint.

        :return: A tuple containing a bool indicates whether the joint is cyclic
            (the joint varies between -pi and +pi in a cyclic manner), and a
            list containing the interval of the joint. interval[0] is the joint
            minimum allowed value, interval[1] is the joint range (the maximum
            allowed value is interval[0]+interval[1]). When the joint is
            "cyclic", then the interval parameters don't have any meaning.
        """
        cyclic, interval = sim.simGetJointInterval(self._handle)
        return cyclic, interval

    def set_joint_interval(self, cyclic: bool, interval: List[float]) -> None:
        """Sets the interval parameters of a joint (i.e. range values).

        The attributes or interval parameters might have no effect, depending
        on the joint-type.

        :param cyclic: Indicates whether the joint is cyclic.
            Only revolute joints with a pitch of 0 can be cyclic.
        :param interval: Interval of the joint. interval[0] is the joint minimum
            allowed value, interval[1] is the joint range (i.e. the maximum
            allowed value is interval[0]+interval[1]).
        """
        sim.simSetJointInterval(self._handle, cyclic, interval)

    def get_joint_upper_velocity_limit(self) -> float:
        """Gets the joints upper velocity limit.

        :return: The upper velocity limit.
        """
        return sim.simGetObjectFloatParameter(
            self._handle, sim.sim_jointfloatparam_upper_limit)

    def is_control_loop_enabled(self) -> bool:
        """Gets whether the control loop is enable.

        :return: True if the control loop is enabled.
        """
        return sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_jointintparam_ctrl_enabled)

    def set_control_loop_enabled(self, value: bool) -> None:
        """Sets whether the control loop is enable.

        :param value: The new value for the control loop state.
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_jointintparam_ctrl_enabled, value)

    def is_motor_enabled(self) -> bool:
        """Gets whether the motor is enable.

        :return: True if the motor is enabled.
        """
        return sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_jointintparam_motor_enabled)

    def set_motor_enabled(self, value: bool) -> None:
        """Sets whether the motor is enable.

        :param value: The new value for the motor state.
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_jointintparam_motor_enabled, value)

    def is_motor_locked_at_zero_velocity(self) -> bool:
        """Gets if the motor is locked when target velocity is zero.

        When enabled in velocity mode and its target velocity is zero, then the
        joint is locked in place.

        :return: If the motor will be locked at zero velocity.
        """
        return sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_jointintparam_velocity_lock)

    def set_motor_locked_at_zero_velocity(self, value: bool) -> None:
        """Set if the motor is locked when target velocity is zero.

        When enabled in velocity mode and its target velocity is zero, then the
        joint is locked in place.

        :param value: If the motor should be locked at zero velocity.
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_jointintparam_velocity_lock, value)

    def get_joint_mode(self) -> JointMode:
        """Retrieves the operation mode of the joint.

        :return: The joint mode.
        """
        return JointMode(sim.simGetJointMode(self._handle))

    def set_joint_mode(self, value: JointMode) -> None:
        """Sets the operation mode of the joint.

        :param value: The new joint mode value.
        """
        sim.simSetJointMode(self._handle, value.value)


object_type_to_class[ObjectType.JOINT] = Joint
