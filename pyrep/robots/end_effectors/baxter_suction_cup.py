from pyrep.robots.end_effectors.suction_cup import SuctionCup


class BaxterSuctionCup(SuctionCup):

    def __init__(self, count: int = 0):
        super().__init__(count, 'BaxterSuctionCup')
