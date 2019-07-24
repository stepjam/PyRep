from pyrep.robots.arms.arm import Arm


class youBot(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'YouBot', 5)
