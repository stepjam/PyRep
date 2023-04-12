from pyrep.robots.arms.arm import Arm


class XArm7(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'xarm', 7)
