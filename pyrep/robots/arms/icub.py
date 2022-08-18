from pyrep.robots.arms.arm import Arm

class ICUB(Arm):

        def __init__(self, count: int = 0):
            super().__init__(count, 'ICUB', num_joints=55)
