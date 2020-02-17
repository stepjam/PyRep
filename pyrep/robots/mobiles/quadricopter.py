from pyrep.robots.mobiles.drone_base import Drone_base


class Quadricopter(Drone_base):
    def __init__(self, count: int = 0, num_propeller:int = 4):
        super().__init__(count, num_propeller, 'Quadricopter')