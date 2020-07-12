from pyrep.robots.arms.arm import Arm


class NAOLeftArm(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'NAO_leftArm', 5, base_name='NAO')


class NAORightArm(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'NAO_rightArm', 5, base_name='NAO')
