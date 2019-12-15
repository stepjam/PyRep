from typing import List

from pyrep.backend import sim
from pyrep.const import ObjectType
from pyrep.objects.force_sensor import ForceSensor
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape


class Accelerometer(Object):
    """An object able to measure accelerations that are applied to it.
    """

    def __init__(self, name):
        super().__init__(name)
        self._mass_object = Shape('%s_mass' % (self.get_name()))
        self._sensor = ForceSensor('%s_force_sensor' % (self.get_name()))

    def _get_requested_type(self) -> ObjectType:
        return ObjectType(sim.simGetObjectType(self.get_handle()))

    def read(self) -> List[float]:
        """Reads the acceleration applied to accelerometer.

        :return: A list containing applied accelerations along
            the sensor's x, y and z-axes
        """
        forces, _ = self._sensor.read()
        accel = [force / self._mass_object.get_mass() for force in forces]
        return accel
