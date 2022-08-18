from pyrep.robots.end_effectors.gripper import Gripper


class ICUBGripper(Gripper):

    def __init__(self, count: int = 0):
        super().__init__(count, 'ICUBHand',
                ['ICUBHand_joint1_finger1', 'ICUBHand_joint2_finger1', 
                    'ICUBHand_joint3_finger1', 'ICUBHand_joint4_finger1'
                    ,'ICUBHand_joint1_finger2', 'ICUBHand_joint2_finger2', 
                    'ICUBHand_joint3_finger2', 'ICUBHand_joint4_finger2',
                    'ICUBHand_joint1_finger3', 'ICUBHand_joint2_finger3',
                    'ICUBHand_joint3_finger3', 'ICUBHand_joint4_finger3',
                    'ICUBHand_joint1_finger4', 'ICUBHand_joint2_finger4',
                    'ICUBHand_joint3_finger4', 'ICUBHand_joint4_finger4',
                    'ICUBHand_joint1_finger5', 'ICUBHand_joint2_finger5',
                    'ICUBHand_joint3_finger5', 'ICUBHand_joint4_finger5'])
