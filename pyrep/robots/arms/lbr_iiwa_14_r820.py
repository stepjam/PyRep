from pyrep.robots.arms.arm import Arm


class LBRIwaa14R820(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'LBR_iiwa_14_R820', 7)
