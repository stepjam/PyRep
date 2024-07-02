from pyrep.robots.arms.arm import Arm


class BaxterLeft(Arm):

    def __init__(
        self,
        count: int = 0,
        max_velocity: float = 1.0,
        max_acceleration: float = 4.0
    ):
        super().__init__(
            count,
            'Baxter_leftArm',
            7,
            base_name='Baxter',
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
        )


class BaxterRight(Arm):

    def __init__(
        self,
        count: int = 0,
        max_velocity: float = 1.0,
        max_acceleration: float = 4.0
    ):
        super().__init__(
            count,
            'Baxter_rightArm',
            7,
            base_name='Baxter',
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
        )