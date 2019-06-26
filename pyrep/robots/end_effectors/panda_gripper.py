from pyrep.robots.end_effectors.gripper import Gripper


class PandaGripper(Gripper):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Panda_gripper',
                         ['Panda_gripper_joint1', 'Panda_gripper_joint2'])
