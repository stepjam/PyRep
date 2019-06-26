from typing import Union
from pyrep.backend import vrep
from pyrep.objects.object import Object
import numpy as np
from pyrep.const import ObjectType


class VisionSensor(Object):
    """A camera-type sensor, reacting to light, colors and images.
    """

    def __init__(self, name_or_handle: Union[str, int]):
        super().__init__(name_or_handle)
        self.resolution = vrep.simGetVisionSensorResolution(self._handle)

    def get_type(self) -> ObjectType:
        return ObjectType.VISION_SENSOR

    def capture_rgb(self) -> np.ndarray:
        """Retrieves the rgb-image of a vision sensor.

        :return: A numpy array of size (width, height, 3)
        """
        return vrep.simGetVisionSensorImage(self._handle, self.resolution)

    def capture_depth(self) -> np.ndarray:
        """Retrieves the depth-image of a vision sensor.

        :return: A numpy array of size (width, height)
        """
        return vrep.simGetVisionSensorDepthBuffer(self._handle, self.resolution)
