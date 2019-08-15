from pyrep.robots.arms.arm import Arm


class Dobot(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Dobot', 4)
