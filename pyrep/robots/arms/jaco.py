from pyrep.robots.arms.arm import Arm


class Jaco(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Jaco', 6)
