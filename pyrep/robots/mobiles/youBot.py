from pyrep.robots.mobiles.mobile import Mobile


class youBot(Mobile):

    def __init__(self, count: int = 0, distance_from_target: float = 0):
        super().__init__(count, 'youBot', 'omnidirectional', None, 4, 6, 0.035, distance_from_target)
