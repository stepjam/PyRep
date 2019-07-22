import math
from typing import List, Union
from pyrep.backend import vrep
from pyrep.objects.object import Object
import numpy as np
from pyrep.const import ObjectType, PerspectiveMode, RenderMode


class VisionSensor(Object):
    """A camera-type sensor, reacting to light, colors and images.
    """

    def __init__(self, name_or_handle: Union[str, int]):
        super().__init__(name_or_handle)
        self.resolution = vrep.simGetVisionSensorResolution(self._handle)

    @staticmethod
    def create(resolution: List[int], explicit_handling=False,
               perspective_mode=True, show_volume_not_detecting=True,
               show_volume_detecting=True, passive=False,
               use_local_lights=False, show_fog=True,
               near_clipping_plane=1e-2, far_clipping_plane=10.0,
               view_angle=60.0, ortho_size=1.0, sensor_size=None,
               render_mode=RenderMode.OPENGL3,
               position=None, orientation=None) -> 'VisionSensor':
        """ Create a Vision Sensor

        :param resolution: List of the [x, y] resolution.
        :param explicit_handling: Sensor will be explicitly handled.
        :param perspective_mode: Sensor will be operated in Perspective Mode.
            Orthographic mode if False.
        :param show_volume_not_detecting: Sensor volume will be shown when not
            detecting anything.
        :param show_volume_detecting: Sensor will be shown when detecting.
        :param passive: Sensor will be passive (use an external image).
        :param use_local_lights: Sensor will use local lights.
        :param show_fog: Sensor will show fog (if enabled).
        :param near_clipping_plane: Near clipping plane.
        :param far_clipping_plane: Far clipping plane.
        :param view_angle: Perspective angle (in degrees) if in Perspective Mode.
        :param ortho_size: Orthographic projection size [m] if in Orthographic
            Mode.
        :param sensor_size: Size [x, y, z] of the Vision Sensor object.
        :param render_mode: Sensor rendering mode, one of:
                RenderMode.OPENGL
                RenderMode.OPENGL_AUXILIARY
                RenderMode.OPENGL_COLOR_CODED
                RenderMode.POV_RAY
                RenderMode.EXTERNAL
                RenderMode.EXTERNAL_WINDOWED
                RenderMode.OPENGL3
                RenderMode.OPENGL3_WINDOWED
        :param position: The [x, y, z] position, if specified.
        :param orientation: The [x, y, z] orientation in radians, if specified.
        :return: The created Vision Sensor.
        """
        options = 0
        if explicit_handling:
            options |= 1
        if perspective_mode:
            options |= 2
        if not show_volume_not_detecting:
            options |= 4
        if not show_volume_detecting:
            options |= 8
        if passive:
            options |= 16
        if use_local_lights:
            options |= 32
        if not show_fog:
            options |= 64

        int_params = [
            resolution[0],  # 0
            resolution[1],  # 1
            0,              # 2
            0               # 3
        ]

        if sensor_size is None:
            sensor_size = [0.01, 0.01, 0.03]

        float_params = [
            near_clipping_plane,    # 0
            far_clipping_plane,     # 1
            math.radians(view_angle) if perspective_mode else ortho_size,  # 2
            sensor_size[0],         # 3
            sensor_size[1],         # 4
            sensor_size[2],         # 5
            0.0,                    # 6
            0.0,                    # 7
            0.0,                    # 8
            0.0,                    # 9
            0.0,                    # 10
        ]

        vs = VisionSensor(
            vrep.simCreateVisionSensor(options, int_params, float_params, None)
        )
        vs.set_render_mode(render_mode)
        if position is not None:
            vs.set_position(position)
        if orientation is not None:
            vs.set_orientation(orientation)
        return vs

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

    def get_resolution(self) -> List[int]:
        """ Return the Sensor's resolution.

        :return: Resolution [x, y]
        """
        return vrep.simGetVisionSensorResolution(self._handle)

    def set_resolution(self, resolution: List[int]) -> None:
        """ Set the Sensor's resolution.

        :param resolution: New resolution [x, y]
        """
        vrep.simSetObjectInt32Parameter(
            self._handle, vrep.sim_visionintparam_resolution_x, resolution[0]
        )
        vrep.simSetObjectInt32Parameter(
            self._handle, vrep.sim_visionintparam_resolution_y, resolution[1]
        )
        self.resolution = resolution

    def get_perspective_mode(self) -> PerspectiveMode:
        """ Retreive the Sensor's perspective mode.

        :return: The current PerspectiveMode.
        """
        perspective_mode = vrep.simGetObjectInt32Parameter(
            self._handle, vrep.sim_visionintparam_perspective_mode
        )
        return PerspectiveMode(perspective_mode)

    def set_perspective_mode(self, perspective_mode: PerspectiveMode) -> None:
        """ Set the Sensor's perspective mode.

        :param perspective_mode: The new perspective mode, one of:
            PerspectiveMode.ORTHOGRAPHIC
            PerspectiveMode.PERSPECTIVE
        """
        vrep.simSetObjectInt32Parameter(
            self._handle, vrep.sim_visionintparam_perspective_mode,
            perspective_mode.value
        )

    def get_render_mode(self) -> RenderMode:
        """ Retrieves the Sensor's rendering mode

        :return: RenderMode for the current rendering mode.
        """
        render_mode = vrep.simGetObjectInt32Parameter(
            self._handle, vrep.sim_visionintparam_render_mode
        )
        return RenderMode(render_mode)

    def set_render_mode(self, render_mode: RenderMode) -> None:
        """ Set the Sensor's rendering mode

        :param render_mode: The new sensor rendering mode, one of:
            RenderMode.OPENGL
            RenderMode.OPENGL_AUXILIARY
            RenderMode.OPENGL_COLOR_CODED
            RenderMode.POV_RAY
            RenderMode.EXTERNAL
            RenderMode.EXTERNAL_WINDOWED
            RenderMode.OPENGL3
            RenderMode.OPENGL3_WINDOWED
        """
        vrep.simSetObjectInt32Parameter(
            self._handle, vrep.sim_visionintparam_render_mode,
            render_mode.value
        )

    def get_perspective_angle(self) -> float:
        """ Get the Sensor's perspective angle.

        :return: The sensor's perspective angle (in degrees).
        """
        return math.degrees(vrep.simGetObjectFloatParameter(
            self._handle, vrep.sim_visionfloatparam_perspective_angle
        ))

    def set_perspective_angle(self, angle: float) -> None:
        """ Set the Sensor's perspective angle.

        :param angle: New perspective angle (in degrees)
        """
        vrep.simSetObjectFloatParameter(
            self._handle, vrep.sim_visionfloatparam_perspective_angle,
            math.radians(angle)
        )

    def get_orthographic_size(self) -> float:
        """ Get the Sensor's orthographic size.

        :return: The sensor's orthographic size (in metres).
        """
        return vrep.simGetObjectFloatParameter(
            self._handle, vrep.sim_visionfloatparam_ortho_size
        )

    def set_orthographic_size(self, ortho_size: float) -> None:
        """ Set the Sensor's orthographic size.

        :param angle: New orthographic size (in metres)
        """
        vrep.simSetObjectFloatParameter(
            self._handle, vrep.sim_visionfloatparam_ortho_size, ortho_size
        )
