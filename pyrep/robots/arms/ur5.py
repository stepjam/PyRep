from pyrep.robots.arms.arm import Arm


class UR5(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'UR5', 6)
