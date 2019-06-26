from pyrep.robots.arms.arm import Arm


class Mico(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Mico', 6)
