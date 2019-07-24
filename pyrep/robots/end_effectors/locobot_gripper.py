from pyrep.robots.end_effectors.gripper import Gripper


class LoCoBotGripper(Gripper):

    def __init__(self, count: int = 0):
        super().__init__(count, 'LoCoBotGripper',
                         ['LoCoBotGripper_joint1', 'LoCoBotGripper_joint2'])
