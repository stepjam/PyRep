from pyrep.robots.end_effectors.hand import Hand


class NAOHand(Hand):

    def __init__(self, count: int = 0):
        super().__init__(count, 'NAOHand',
                         ['NAOHand_leftJoint1',
                          'NAOHand_leftJoint2',
                          'NAOHand_leftJoint3',
                          'NAOHand_rightJoint1',
                          'NAOHand_rightJoint2',
                          'NAOHand_rightJoint3',
                          'NAOHand_thumb1',
                          'NAOHand_thumb2'])
