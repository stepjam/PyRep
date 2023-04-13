from pyrep.robots.end_effectors.gripper import Gripper


class Z1Gripper(Gripper):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Z1Gripper',
                         ['Z1Gripper_Joint'])
