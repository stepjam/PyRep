from typing import List
from pyrep.objects.force_sensor import ForceSensor
from pyrep.robots.robot_component import RobotComponent


class Hand(RobotComponent):
    """Represents all types of end-effectors, e.g. grippers.
    """

    def __init__(self, count: int, name: str, joint_names: List[str]):
        super().__init__(count, name, joint_names)
        suffix = '' if count == 0 else '#%d' % (count - 1)

        self._touch_sensors = []
        i = 0
        while True:
            fname = '%s_touchSensor%d%s' % (name, i, suffix)
            if not ForceSensor.exists(fname):
                break
            self._touch_sensors.append(ForceSensor(fname))
            i += 1

    def get_touch_sensor_forces(self) -> List[List[float]]:
        if len(self._touch_sensors) == 0:
            raise NotImplementedError('No touch sensors for this robot!')
        return [ts.read()[0] for ts in self._touch_sensors]
