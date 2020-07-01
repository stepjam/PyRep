import math
from typing import List, Union, Sequence
from pyrep.backend import sim
from pyrep.objects.object import Object, object_type_to_class
import numpy as np
from pyrep.const import ObjectType, PerspectiveMode, RenderMode


class VisionSensor(Object):
    """A camera-type sensor, reacting to light, colors and images.
    """

    def __init__(self, name_or_handle: Union[str, int]):
        super().__init__(name_or_handle)
        self.resolution = sim.simGetVisionSensorResolution(self._handle)

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
            sim.simCreateVisionSensor(options, int_params, float_params, None)
        )
        vs.set_render_mode(render_mode)
        if position is not None:
            vs.set_position(position)
        if orientation is not None:
            vs.set_orientation(orientation)
        return vs

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.VISION_SENSOR

    def handle_explicitly(self) -> None:
        """Handle sensor explicitly.

          This enables capturing image (e.g., capture_rgb())
          without PyRep.step().
        """
        if not self.get_explicit_handling():
            raise RuntimeError('The explicit_handling is disabled. '
                               'Call set_explicit_handling(value=1) to enable explicit_handling first.')
        sim.simHandleVisionSensor(self._handle)

    def capture_rgb(self) -> np.ndarray:
        """Retrieves the rgb-image of a vision sensor.

        :return: A numpy array of size (width, height, 3)
        """
        return sim.simGetVisionSensorImage(self._handle, self.resolution)

    def capture_depth(self, in_meters=False) -> np.ndarray:
        """Retrieves the depth-image of a vision sensor.

        :param in_meters: Whether the depth should be returned in meters.
        :return: A numpy array of size (width, height)
        """
        return sim.simGetVisionSensorDepthBuffer(
            self._handle, self.resolution, in_meters)

    def get_resolution(self) -> List[int]:
        """ Return the Sensor's resolution.

        :return: Resolution [x, y]
        """
        return sim.simGetVisionSensorResolution(self._handle)

    def set_resolution(self, resolution: List[int]) -> None:
        """ Set the Sensor's resolution.

        :param resolution: New resolution [x, y]
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_resolution_x, resolution[0]
        )
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_resolution_y, resolution[1]
        )
        self.resolution = resolution

    def get_perspective_mode(self) -> PerspectiveMode:
        """ Retrieve the Sensor's perspective mode.

        :return: The current PerspectiveMode.
        """
        perspective_mode = sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_perspective_operation,
        )
        return PerspectiveMode(perspective_mode)

    def set_perspective_mode(self, perspective_mode: PerspectiveMode) -> None:
        """ Set the Sensor's perspective mode.

        :param perspective_mode: The new perspective mode, one of:
            PerspectiveMode.ORTHOGRAPHIC
            PerspectiveMode.PERSPECTIVE
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_perspective_operation,
            perspective_mode.value
        )

    def get_render_mode(self) -> RenderMode:
        """ Retrieves the Sensor's rendering mode

        :return: RenderMode for the current rendering mode.
        """
        render_mode = sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_render_mode
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
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_render_mode,
            render_mode.value
        )

    def get_windowed_size(self) -> Sequence[int]:
        """Get the size of windowed rendering.

        :return: The (x, y) resolution of the window. 0 for full-screen.
        """
        size_x = sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_windowed_size_x)
        size_y = sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_windowed_size_y)
        return size_x, size_y

    def set_windowed_size(self, resolution: Sequence[int] = (0, 0)) -> None:
        """Set the size of windowed rendering.

        :param resolution: The (x, y) resolution of the window.
            0 for full-screen.
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_windowed_size_x,
            resolution[0])
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_windowed_size_y,
            resolution[1])

    def get_perspective_angle(self) -> float:
        """ Get the Sensor's perspective angle.

        :return: The sensor's perspective angle (in degrees).
        """
        return math.degrees(sim.simGetObjectFloatParameter(
            self._handle, sim.sim_visionfloatparam_perspective_angle
        ))

    def set_perspective_angle(self, angle: float) -> None:
        """ Set the Sensor's perspective angle.

        :param angle: New perspective angle (in degrees)
        """
        sim.simSetObjectFloatParameter(
            self._handle, sim.sim_visionfloatparam_perspective_angle,
            math.radians(angle)
        )

    def get_orthographic_size(self) -> float:
        """ Get the Sensor's orthographic size.

        :return: The sensor's orthographic size (in metres).
        """
        return sim.simGetObjectFloatParameter(
            self._handle, sim.sim_visionfloatparam_ortho_size
        )

    def set_orthographic_size(self, ortho_size: float) -> None:
        """ Set the Sensor's orthographic size.

        :param angle: New orthographic size (in metres)
        """
        sim.simSetObjectFloatParameter(
            self._handle, sim.sim_visionfloatparam_ortho_size, ortho_size
        )

    def get_near_clipping_plane(self) -> float:
        """ Get the Sensor's near clipping plane.

        :return: Near clipping plane (metres)
        """
        return sim.simGetObjectFloatParameter(
            self._handle, sim.sim_visionfloatparam_near_clipping
        )

    def set_near_clipping_plane(self, near_clipping: float) -> None:
        """ Set the Sensor's near clipping plane.

        :param near_clipping: New near clipping plane (in metres)
        """
        sim.simSetObjectFloatParameter(
            self._handle, sim.sim_visionfloatparam_near_clipping, near_clipping
        )

    def get_far_clipping_plane(self) -> float:
        """ Get the Sensor's far clipping plane.

        :return: Near clipping plane (metres)
        """
        return sim.simGetObjectFloatParameter(
            self._handle, sim.sim_visionfloatparam_far_clipping
        )

    def set_far_clipping_plane(self, far_clipping: float) -> None:
        """ Set the Sensor's far clipping plane.

        :param far_clipping: New far clipping plane (in metres)
        """
        sim.simSetObjectFloatParameter(
            self._handle, sim.sim_visionfloatparam_far_clipping, far_clipping
        )

    def set_entity_to_render(self, entity_to_render: int) -> None:
        """ Set the entity to render to the Sensor, this can be an object or more usefully a collection.
        -1 to render all objects in scene.

        :param entity_to_render: Handle of the entity to render
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_entity_to_render, entity_to_render
        )

    def get_entity_to_render(self) -> None:
        """ Get the entity to render to the Sensor, this can be an object or more usefully a collection.
        -1 if all objects in scene are rendered.

        :return: Handle of the entity to render
        """
        return sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_visionintparam_entity_to_render
        )


object_type_to_class[ObjectType.VISION_SENSOR] = VisionSensor
