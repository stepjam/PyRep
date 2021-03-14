from typing import List

from pyrep.objects import Joint
from pyrep.robots.end_effectors.gripper import Gripper, POSITION_ERROR
import numpy as np

POSITION_ERROR = 0.00001


class Robotiq85Gripper(Gripper):

    def __init__(self, count: int = 0):
        super().__init__(count, 'ROBOTIQ_85', ['ROBOTIQ_85_active1',
                                               'ROBOTIQ_85_active2'])
        self._open_amount_query_joints = [
            Joint(jname) for jname in [
                'ROBOTIQ_85_Rjoint1', 'ROBOTIQ_85_Ljoint1']]

    def actuate(self, amount: float, velocity: float) -> bool:
        if amount != 0.0 and amount != 1.0:
            raise ValueError(
                'This gripper currently only supports fully open or closed.')

        current_positions = self.get_joint_positions()
        vel = velocity if amount == 1.0 else -velocity
        done = True
        for i, (j, cur, prev) in enumerate(zip(
                self.joints, current_positions,
                self._prev_positions)):
            # Check if the joint has moved much
            not_moving = (prev is not None and
                          np.fabs(cur - prev) < POSITION_ERROR)
            if not_moving:
                j.set_joint_target_velocity(0)
                continue
            done = False
            self._prev_vels[i] = vel  # type: ignore
            j.set_joint_target_velocity(vel)
        self._prev_positions = current_positions  # type: ignore
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
        joint_intervals = np.array([j.get_joint_interval()[1]
                     for j in self._open_amount_query_joints])
        joint_range = joint_intervals[:, 1] - joint_intervals[:, 0]
        # Flip open amount
        return list(1. - np.clip((np.array(
            [j.get_joint_position() for j in self._open_amount_query_joints]
        ) - joint_intervals[:, 0]) / joint_range, 0.0, 1.0))
