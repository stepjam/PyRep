from pyrep.robots.arms.arm import Arm


class LoCoBotArm(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'LoCoBotArm', 5)
