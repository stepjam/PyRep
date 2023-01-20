from pyrep.robots.arms.arm import Arm


class xarm(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'xarm', 13)
