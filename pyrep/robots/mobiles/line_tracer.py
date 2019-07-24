from pyrep.robots.mobiles.mobile_base import MobileBase


class LineTracer(MobileBase):
    def __init__(self, count: int = 0):
        super().__init__(count, 2, 'LineTracer')
