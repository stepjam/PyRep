import numpy as np
from typing import List, Sequence

from pyrep.backend import sim, utils
from pyrep.objects.object import Object
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.const import PYREP_SCRIPT_TYPE, RenderMode, ObjectType

MIN_DIVISOR = 1e-12


class SphericalVisionSensor(Object):
    """An object able capture 360 degree omni-directional images.
    """

    def __init__(self, name):
        super().__init__(name)

        # final omni-directional sensors
        self._sensor_depth = VisionSensor('%s_sensorDepth' % (self.get_name()))
        self._sensor_rgb = VisionSensor('%s_sensorRGB' % (self.get_name()))

        # directed sub-sensors
        names = ['front', 'top', 'back', 'bottom', 'left', 'right']
        self._six_sensors = [VisionSensor('%s_%s' % (self.get_name(), n)) for n in names]
        self._front = self._six_sensors[0]

        # directed sub-sensor handles list
        self._six_sensor_handles = [vs.get_handle() for vs in self._six_sensors]

        # set all to explicit handling
        self._set_all_to_explicit_handling()

        # assert sensor properties are matching
        self._assert_matching_resolutions()
        self._assert_matching_render_modes()
        self._assert_matching_entities_to_render()
        self._assert_matching_window_sizes()
        self._assert_matching_near_clipping_planes()
        self._assert_matching_far_clipping_planes()

        # omni resolution
        self._omni_resolution = self._sensor_rgb.get_resolution()

        # near and far clipping plane
        self._near_clipping_plane = self.get_near_clipping_plane()
        self._far_clipping_plane = self.get_far_clipping_plane()
        self._clipping_plane_diff = self._far_clipping_plane - self._near_clipping_plane

        # projective cam region image
        self._planar_to_radial_depth_scalars = self._get_planar_to_radial_depth_scalars()

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

    def _assert_matching_near_clipping_planes(self):
        front_sensor_near = self._front.get_near_clipping_plane()
        for sensor in self._six_sensors[1:]:
            assert sensor.get_near_clipping_plane() == front_sensor_near

    def _assert_matching_far_clipping_planes(self):
        front_sensor_far = self._front.get_far_clipping_plane()
        for sensor in self._six_sensors[1:]:
            assert sensor.get_far_clipping_plane() == front_sensor_far

    def _get_requested_type(self) -> ObjectType:
        return ObjectType(sim.simGetObjectType(self.get_handle()))

    @staticmethod
    def _create_uniform_pixel_coords_image(image_dims):
        pixel_x_coords = np.reshape(np.tile(np.arange(image_dims[1]), [image_dims[0]]),
                                    (image_dims[0], image_dims[1], 1)).astype(np.float32)
        pixel_y_coords_ = np.reshape(np.tile(np.arange(image_dims[0]), [image_dims[1]]),
                                     (image_dims[1], image_dims[0], 1)).astype(np.float32)
        pixel_y_coords = np.transpose(pixel_y_coords_, (1, 0, 2))
        return np.concatenate((pixel_x_coords, pixel_y_coords), -1)

    def _get_planar_to_radial_depth_scalars(self):

        # reference: https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates

        # polar coord image
        coord_img = self._create_uniform_pixel_coords_image([self._omni_resolution[1], self._omni_resolution[0]])
        pixels_per_rad = self._omni_resolution[1] / np.pi
        polar_angles = coord_img / pixels_per_rad
        phi = polar_angles[..., 0:1]
        theta = polar_angles[..., 1:2]

        # image borders wrt the 6 projective cameras
        phis_rel = ((phi + np.pi/4) % (np.pi/2)) / (np.pi/2)
        lower_borders = np.arccos(1/((phis_rel*2-1)**2 + 2)**0.5)
        upper_borders = np.pi - lower_borders

        # omni image regions from the 6 projective cameras
        xy_region = np.logical_and(theta <= upper_borders, theta >= lower_borders)
        pos_x = np.logical_and(xy_region, np.logical_or(phi <= 45 * np.pi/180, phi >= (45 + 270) * np.pi/180))
        pos_y = np.logical_and(xy_region,
                               np.logical_and(phi >= 45 * np.pi/180, phi <= (45 + 90) * np.pi/180))
        neg_x = np.logical_and(xy_region,
                               np.logical_and(phi >= (45 + 90) * np.pi/180, phi <= (45 + 180) * np.pi/180))
        neg_y = np.logical_and(xy_region,
                               np.logical_and(phi >= (45 + 180) * np.pi/180, phi <= (45 + 270) * np.pi/180))
        pos_z = theta <= lower_borders
        neg_z = theta >= upper_borders

        # trig terms for conversion
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)

        # reciprocal terms
        recip_sin_theta_cos_phi = 1/(sin_theta * cos_phi + MIN_DIVISOR)
        recip_sin_theta_sin_phi = 1/(sin_theta * sin_phi + MIN_DIVISOR)
        recip_cos_theta = 1/(cos_theta + MIN_DIVISOR)

        # planar to radial depth scalars
        return np.where(pos_x, recip_sin_theta_cos_phi,
                        np.where(neg_x, -recip_sin_theta_cos_phi,
                                 np.where(pos_y, recip_sin_theta_sin_phi,
                                          np.where(neg_y, -recip_sin_theta_sin_phi,
                                                   np.where(pos_z, recip_cos_theta,
                                                            np.where(neg_z, -recip_cos_theta,
                                                                     np.zeros_like(recip_cos_theta)))))))[..., 0]

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

    def capture_depth(self, in_meters=False) -> np.ndarray:
        """Retrieves the depth-image of a spherical vision sensor.

        :param in_meters: Whether the depth should be returned in meters.
        :return: A numpy array of size (width, height)
        """
        planar_depth = self._sensor_depth.capture_rgb()[:, :, 0]*self._clipping_plane_diff + self._near_clipping_plane
        radial_depth = planar_depth * self._planar_to_radial_depth_scalars
        if in_meters:
            return radial_depth
        return (radial_depth - self._near_clipping_plane)/self._clipping_plane_diff

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
        return self._front.get_near_clipping_plane()

    def set_near_clipping_plane(self, near_clipping: float) -> None:
        """ Set the spherical depth sensor's near clipping plane.

        :param near_clipping: New near clipping plane (in metres)
        """
        self._near_clipping_plane = near_clipping
        self._clipping_plane_diff = self._far_clipping_plane - near_clipping
        for sensor in self._six_sensors:
            sensor.set_near_clipping_plane(near_clipping)

    def get_far_clipping_plane(self) -> float:
        """ Get the Sensor's far clipping plane.

        :return: Near clipping plane (metres)
        """
        return self._front.get_far_clipping_plane()

    def set_far_clipping_plane(self, far_clipping: float) -> None:
        """ Set the spherical depth sensor's far clipping plane.

        :param far_clipping: New far clipping plane (in metres)
        """
        self._far_clipping_plane = far_clipping
        self._clipping_plane_diff = far_clipping - self._near_clipping_plane
        for sensor in self._six_sensors:
            sensor.set_far_clipping_plane(far_clipping)

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
