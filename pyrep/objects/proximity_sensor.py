from pyrep.backend import vrep
from pyrep.objects.object import Object
from pyrep.const import ObjectType


class ProximitySensor(Object):
    """Detects objects within a detection volume.

    V-REP supports pyramid-, cylinder-, disk-, cone- and ray-type proximity
    sensors.
    """

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.PROXIMITY_SENSOR

    def is_detected(self, obj: Object) -> bool:
        """Checks whether the proximity sensor detects the indicated object.

        :param obj: The object to detect.
        :return: Bool indicating if the object was detected.
        """
        state, point = vrep.simCheckProximitySensor(
            self._handle, obj.get_handle())
        return state == 1
