from pyrep.robots.end_effectors.gripper import Gripper


class XArmGripper(Gripper):

    def __init__(self, count: int = 0):
        super().__init__(count, 'xarm_gripper',
                         ['xarm_left_inner_knuckle_joint',
                          'xarm_right_inner_knuckle_joint',
                          'xarm_drive_joint',
                          'xarm_left_finger_joint',
                          'xarm_right_outer_knuckle_joint',
                          'xarm_right_finger_joint'])
