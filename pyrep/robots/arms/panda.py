from pyrep.robots.arms.arm import Arm


class Panda(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Panda', 7)
