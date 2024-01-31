from math import sqrt

from pyrep.objects.object import Object, object_type_to_class
from pyrep.const import ObjectType


class ProximitySensor(Object):
    """Detects objects within a detection volume.

    V-REP supports pyramid-, cylinder-, disk-, cone- and ray-type proximity
    sensors.
    """

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.PROXIMITY_SENSOR

    def read(self) -> float:
        """Read the distance between sensor and first detected object. If
        there is no detected object returns -1.0. It can be considered as
        maximum measurable distance of the sensor.

        :return: Float distance to the first detected object
        """
        state, dist, points, obj, norm_vec = self._sim_api.readProximitySensor(
            self._handle
        )
        if state:
            return sqrt(points[0] ** 2 + points[1] ** 2 + points[2] ** 2)
        return -1.0

    def is_detected(self, obj: Object) -> bool:
        """Checks whether the proximity sensor detects the indicated object.

        :param obj: The object to detect.
        :return: Bool indicating if the object was detected.
        """
        res, dist, point, obj, norm_vec = self._sim_api.checkProximitySensor(
            self._handle, obj.get_handle()
        )
        return res == 1


object_type_to_class[ObjectType.PROXIMITY_SENSOR] = ProximitySensor
