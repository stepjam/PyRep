from pyrep.robots.arms.arm import Arm


class LaikagoLeftFront(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Laikago_leftfront', 3, base_name='Laikago')


class LaikagoLeftRear(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Laikago_leftrear', 3, base_name='Laikago')

class LaikagoRightFront(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Laikago_rightfront', 3, base_name='Laikago')

class LaikagoRightRear(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'Laikago_rightrear', 3, base_name='Laikago')
