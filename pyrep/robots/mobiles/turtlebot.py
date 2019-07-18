from pyrep.robots.mobiles.nonholonomic_base import NonHolonomicBase


class turtlebot(NonHolonomicBase):
    def __init__(self, count: int = 0, distance_from_target: float = 0):
        super().__init__(
            count, 2, distance_from_target, 'turtlebot', 4, 6, 0.035)
