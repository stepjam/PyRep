from pyrep.objects import Object
from pyrep.robots.end_effectors.gripper import Gripper


class BaxterGripper(Gripper):
    def __init__(self, count: int = 0, proxy: Object = None):
        super().__init__(
            count,
            "BaxterGripper",
            ["BaxterGripper_leftJoint", "BaxterGripper_rightJoint"],
            proxy=proxy,
        )
