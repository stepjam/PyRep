from typing import List
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.force_sensor import ForceSensor
from pyrep.robots.robot_component import RobotComponent
import numpy as np

POSITION_ERROR = 0.001


class Gripper(RobotComponent):
    """Represents all types of end-effectors, e.g. grippers.
    """

    def __init__(self, count: int, name: str, joint_names: List[str]):
        super().__init__(count, name, joint_names)
        prox_name = '%s_attachProxSensor' % name
        attach_name = '%s_attachPoint' % name
        prox_name = prox_name if count == 0 else prox_name + '%d' % (count - 1)
        attach_name = attach_name if count == 0 else attach_name + '%d' % (count - 1)
        self._proximity_sensor = ProximitySensor(prox_name)
        self._attach_point = ForceSensor(attach_name)
        self._old_parents = []
        self._grasped_objects = []
        self._prev_positions = [None] * len(joint_names)
        self._prev_vels = [None] * len(joint_names)  # Used to stop oscillating

    def grasp(self, obj: Object) -> bool:
        """Grasp object if it is detected.

        Note: The does not actuate the gripper, but instead simply attaches the
        detected object to the gripper to 'fake' a grasp.

        :param obj: The object to grasp if detected.
        :return: True if the object was detected/grasped.
        """
        detected = self._proximity_sensor.is_detected(obj)
        if detected:
            self._grasped_objects.append(obj)
            self._old_parents.append(obj.get_parent())
            obj.set_parent(self._attach_point, keep_in_place=True)
        return detected

    def release(self) -> None:
        """Release any grasped objects.

        Note: The does not actuate the gripper, but instead simply detaches any
        grasped objects.
        """
        for grasped_obj, old_parent in zip(
                self._grasped_objects, self._old_parents):
            # Check if the object still exists
            if grasped_obj.still_exists() and old_parent.still_exists():
                grasped_obj.set_parent(old_parent, keep_in_place=True)
        self._grasped_objects = []
        self._old_parents = []

    def get_grasped_objects(self) -> List[Object]:
        """Gets the objects that are currently grasped.

        :return: A list of grasped objects.
        """
        return self._grasped_objects

    def actuate(self, amount: float, velocity: float) -> bool:
        """Actuate the gripper, but return after each simulation step.

        The functions attempts to open/close the gripper according to 'amount',
        where 1 represents open, and 0 represents close. The user should
        iteratively call this function until it returns True.

        This is a convenience method. If you would like direct control of the
        gripper, use the :py:class:`RobotComponent` methods instead.

        For some grippers, this method will need to be overridden.

        :param amount: A float between 0 and 1 representing the gripper open
            state. 1 means open, whilst 0 means closed.
        :param velocity: The velocity to apply to the gripper joints.


        :raises: ValueError if 'amount' is not between 0 and 1.

        :return: True if the gripper has reached its open/closed limits, or if
            the 'max_force' has been exerted.
        """
        if not (0.0 <= amount <= 1.0):
            raise ValueError("'open_amount' should be between 0 and 1.'")
        _, joint_intervals = self.get_joint_intervals()
        joint_intervals = np.array(joint_intervals)

        # Decide on if we need to open or close
        joint_range = joint_intervals[:, 1] - joint_intervals[:, 0]
        target_pos = joint_intervals[:, 0] + (joint_range * amount)

        current_positions = self.get_joint_positions()
        done = True
        for i, (j, target, cur, prev) in enumerate(zip(
                self.joints, target_pos, current_positions,
                self._prev_positions)):
            # Check if the joint has moved much
            not_moving = (prev is not None and
                          np.fabs(cur - prev) < POSITION_ERROR)
            reached_target = np.fabs(target - cur) < POSITION_ERROR
            vel = -velocity if cur - target > 0 else velocity
            oscillating = (self._prev_vels[i] is not None and
                           vel != self._prev_vels[i])
            if not_moving or reached_target or oscillating:
                j.set_joint_target_velocity(0)
                continue
            done = False
            vel = -velocity if cur - target > 0 else velocity
            self._prev_vels[i] = vel
            j.set_joint_target_velocity(vel)
        self._prev_positions = current_positions
        if done:
            self._prev_positions = [None] * self._num_joints
            self._prev_vels = [None] * self._num_joints
            self.set_joint_target_velocities([0.0] * self._num_joints)
        return done

    def get_open_amount(self) -> List[float]:
        """Gets the gripper open state. 1 means open, whilst 0 means closed.

        :return: A list of floats between 0 and 1 representing the gripper open
            state for each joint. 1 means open, whilst 0 means closed.
        """
        _, joint_intervals = self.get_joint_intervals()
        joint_intervals = np.array(joint_intervals)
        joint_range = joint_intervals[:, 1] - joint_intervals[:, 0]
        return list(np.clip((np.array(
            self.get_joint_positions()) - joint_intervals[:, 0]) /
                            joint_range, 0.0, 1.0))
