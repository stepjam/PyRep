from pyrep.robots.arms.arm import Arm


class BaxterLeft(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Baxter_leftArm', 7, base_name='Baxter')


class BaxterRight(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Baxter_rightArm', 7, base_name='Baxter')
