import numpy as np
from typing import List, Sequence

from pyrep.backend import sim, utils
from pyrep.objects.object import Object
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.const import PYREP_SCRIPT_TYPE, RenderMode, ObjectType


class SphericalVisionSensor(Object):
    """An object able capture 360 degree omni-directional images.
    """

    def __init__(self, name):
        super().__init__(name)

        # final omni-directional sensors
        self._sensor_depth = VisionSensor('%s_sensorDepth' % (self.get_name()))
        self._sensor_rgb = VisionSensor('%s_sensorRGB' % (self.get_name()))

        # resolutions
        self._depth_res = self._sensor_depth.get_resolution()
        self._rgb_res = self._sensor_rgb.get_resolution()

        # directed sub-sensors
        self._front = VisionSensor('%s_front' % (self.get_name()))
        self._top = VisionSensor('%s_top' % (self.get_name()))
        self._back = VisionSensor('%s_back' % (self.get_name()))
        self._bottom = VisionSensor('%s_bottom' % (self.get_name()))
        self._left = VisionSensor('%s_left' % (self.get_name()))
        self._right = VisionSensor('%s_right' % (self.get_name()))

        # directed sub-sensors list
        self._six_sensors = list()
        self._six_sensors.append(self._front)
        self._six_sensors.append(self._top)
        self._six_sensors.append(self._back)
        self._six_sensors.append(self._bottom)
        self._six_sensors.append(self._left)
        self._six_sensors.append(self._right)

        # directed sub-sensor handles list
        self._six_sensor_handles = list()
        self._six_sensor_handles.append(self._front._handle)
        self._six_sensor_handles.append(self._top._handle)
        self._six_sensor_handles.append(self._back._handle)
        self._six_sensor_handles.append(self._bottom._handle)
        self._six_sensor_handles.append(self._left._handle)
        self._six_sensor_handles.append(self._right._handle)

        # set all to explicit handling
        self._set_all_to_explicit_handling()

        # assert sensor properties are matching
        self._assert_matching_resolutions()
        self._assert_matching_render_modes()
        self._assert_matching_entities_to_render()
        self._assert_matching_window_sizes()

    # Private #
    # --------#

    def _set_all_to_explicit_handling(self):
        self._sensor_depth.set_explicit_handling(1)
        self._sensor_rgb.set_explicit_handling(1)
        for sensor in self._six_sensors:
            sensor.set_explicit_handling(1)

    def _assert_matching_resolutions(self):
        assert self._sensor_depth.get_resolution() == self._sensor_rgb.get_resolution()
        front_sensor_res = self._front.get_resolution()
        for sensor in self._six_sensors[1:]:
            assert sensor.get_resolution() == front_sensor_res

    def _assert_matching_render_modes(self):
        assert self._sensor_depth.get_render_mode() == self._sensor_rgb.get_render_mode()
        front_sensor_render_mode = self._front.get_render_mode()
        for sensor in self._six_sensors[1:]:
            assert sensor.get_render_mode() == front_sensor_render_mode

    def _assert_matching_entities_to_render(self):
        assert self._sensor_depth.get_entity_to_render() == self._sensor_rgb.get_entity_to_render()
        front_sensor_entity_to_render = self._front.get_entity_to_render()
        for sensor in self._six_sensors[1:]:
            assert sensor.get_entity_to_render() == front_sensor_entity_to_render

    def _assert_matching_window_sizes(self):
        assert self._sensor_depth.get_windowed_size() == self._sensor_rgb.get_windowed_size()

    def _get_requested_type(self) -> ObjectType:
        return ObjectType(sim.simGetObjectType(self.get_handle()))

    # Public #
    # -------#

    def handle_explicitly(self) -> None:
        """Handle spherical vision sensor explicitly.

          This enables capturing image (e.g., capture_rgb())
          without PyRep.step().
        """
        utils.script_call('handleSpherical@PyRep', PYREP_SCRIPT_TYPE,
                          [self._sensor_depth._handle, self._sensor_rgb._handle] + self._six_sensor_handles, [], [], [])

    def capture_rgb(self) -> np.ndarray:
        """Retrieves the rgb-image of a spherical vision sensor.

        :return: A numpy array of size (width, height, 3)
        """
        return self._sensor_rgb.capture_rgb()

    def capture_depth(self) -> np.ndarray:
        """Retrieves the depth-image of a spherical vision sensor.

        :return: A numpy array of size (width, height)
        """
        return self._sensor_depth.capture_rgb()[:, :, 0]

    def get_resolution(self) -> List[int]:
        """ Return the spherical vision sensor's resolution.

        :return: Resolution [x, y]
        """
        return self._sensor_depth.get_resolution()

    def set_resolution(self, resolution: List[int]) -> None:
        """ Set the spherical vision sensor's resolution.

        :param resolution: New resolution [x, y]
        """
        if resolution[0] != 2*resolution[1]:
            raise Exception('Spherical vision sensors must have an X resolution 2x the Y resolution')
        if resolution[1] % 2 != 0:
            raise Exception('Spherical vision sensors must have an even Y resolution')
        box_sensor_resolution = [int(resolution[1]/2), int(resolution[1]/2)]
        for sensor in self._six_sensors:
            sensor.set_resolution(box_sensor_resolution)
        self._sensor_depth.set_resolution(resolution)
        self._sensor_rgb.set_resolution(resolution)

    def get_render_mode(self) -> RenderMode:
        """ Retrieves the spherical vision sensor's rendering mode

        :return: RenderMode for the current rendering mode.
        """
        return self._sensor_depth.get_render_mode()

    def set_render_mode(self, render_mode: RenderMode) -> None:
        """ Set the spherical vision sensor's rendering mode

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
        self._sensor_depth.set_render_mode(render_mode)
        self._sensor_rgb.set_render_mode(render_mode)
        for sensor in self._six_sensors:
            sensor.set_render_mode(render_mode)

    def get_windowed_size(self) -> Sequence[int]:
        """Get the size of windowed rendering for the omni-directional image.

        :return: The (x, y) resolution of the window. 0 for full-screen.
        """
        return self._sensor_depth.get_windowed_size()

    def set_windowed_size(self, resolution: Sequence[int] = (0, 0)) -> None:
        """Set the size of windowed rendering for the omni-directional image.

        :param resolution: The (x, y) resolution of the window.
            0 for full-screen.
        """
        self._sensor_depth.set_windowed_size(resolution)
        self._sensor_rgb.set_windowed_size(resolution)

    def get_near_clipping_plane(self) -> float:
        """ Get the spherical depth sensor's near clipping plane.

        :return: Near clipping plane (metres)
        """
        return self._sensor_depth.get_near_clipping_plane()

    def set_near_clipping_plane(self, near_clipping: float) -> None:
        """ Set the spherical depth sensor's near clipping plane.

        :param near_clipping: New near clipping plane (in metres)
        """
        self._sensor_depth.set_near_clipping_plane(near_clipping)

    def get_far_clipping_plane(self) -> float:
        """ Get the Sensor's far clipping plane.

        :return: Near clipping plane (metres)
        """
        return self._sensor_depth.get_far_clipping_plane()

    def set_far_clipping_plane(self, far_clipping: float) -> None:
        """ Set the spherical depth sensor's far clipping plane.

        :param far_clipping: New far clipping plane (in metres)
        """
        self._sensor_depth.set_far_clipping_plane(far_clipping)

    def set_entity_to_render(self, entity_to_render: int) -> None:
        """ Set the entity to render to the spherical vision sensor,
        this can be an object or more usefully a collection. -1 to render all objects in scene.

        :param entity_to_render: Handle of the entity to render
        """
        self._sensor_depth.set_entity_to_render(entity_to_render)
        self._sensor_rgb.set_entity_to_render(entity_to_render)

    def get_entity_to_render(self) -> None:
        """ Get the entity to render to the spherical vision sensor,
        this can be an object or more usefully a collection. -1 if all objects in scene are rendered.

        :return: Handle of the entity to render
        """
        return self._sensor_depth.get_entity_to_render()
