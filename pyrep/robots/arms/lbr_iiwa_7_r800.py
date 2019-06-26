from pyrep.robots.arms.arm import Arm


class LBRIwaa7R800(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'LBR_iiwa_7_R800', 7)
