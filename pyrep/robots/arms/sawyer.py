from pyrep.robots.arms.arm import Arm


class Sawyer(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Sawyer', 7)
