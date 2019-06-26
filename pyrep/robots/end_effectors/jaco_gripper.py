from pyrep.robots.end_effectors.gripper import Gripper


class JacoGripper(Gripper):

    def __init__(self, count: int = 0):
        super().__init__(count, 'JacoHand',
                         ['JacoHand_joint1_finger1',
                          'JacoHand_joint1_finger2',
                          'JacoHand_joint1_finger3'])
