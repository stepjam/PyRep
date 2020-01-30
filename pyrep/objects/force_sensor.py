from typing import Tuple, List
from pyrep.backend import sim
from pyrep.objects.object import Object, object_type_to_class
from pyrep.const import ObjectType


class ForceSensor(Object):
    """An object able to measure forces and torques that are applied to it.
    """

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.FORCE_SENSOR

    def read(self) -> Tuple[List[float], List[float]]:
        """Reads the force and torque applied to a force sensor.

        :return: A tuple containing the applied forces along the
            sensor's x, y and z-axes, and the torques along the
            sensor's x, y and z-axes.
        """
        _, forces, torques = sim.simReadForceSensor(self._handle)
        return forces, torques


object_type_to_class[ObjectType.FORCE_SENSOR] = ForceSensor
