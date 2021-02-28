from pyrep.robots.arms.arm import Arm

class KR6(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'KR6', 6)
