from pyrep.robots.arms.arm import Arm


class Kinova3(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Kinova3', 7)
