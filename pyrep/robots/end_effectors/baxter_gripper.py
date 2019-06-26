from pyrep.robots.end_effectors.gripper import Gripper


class BaxterGripper(Gripper):

    def __init__(self, count: int = 0):
        super().__init__(count, 'BaxterGripper',
                         ['BaxterGripper_leftJoint',
                          'BaxterGripper_rightJoint'])
