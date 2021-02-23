from pyrep.robots.arms.arm import Arm


class ABB120(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'ABB120', 6)
