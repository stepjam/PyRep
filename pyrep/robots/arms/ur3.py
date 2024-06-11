from pyrep.robots.arms.arm import Arm


class UR3(Arm):

    def __init__(
        self,
        count: int = 0,
        max_velocity: float = 1.0,
        max_acceleration: float = 4.0
    ):
        super().__init__(
            count,
            'UR3',
            6,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
        )
