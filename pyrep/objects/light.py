import warnings
import numpy as np
from typing import Union
from pyrep.backend import sim_const as simc
from pyrep.const import ObjectType
from pyrep.objects.object import Object, object_type_to_class


class Light(Object):
    """A light.
    """

    def __init__(self, name_or_handle: Union[str, int]):
        super().__init__(name_or_handle)

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.LIGHT

    # On and Off

    def turn_on(self):
        """ Turn the light on.
        """
        self._sim_api.setLightParameters(self._handle, 1, None, None, None)

    def turn_off(self):
        """ Turn the light off.
        """
        self._sim_api.setLightParameters(self._handle, 0, None, None, None)

    def is_on(self):
        """ Determines whether the light is on.
        return: Boolean
        """
        return self._sim_api.getLightParameters(self._handle)[0]

    def is_off(self):
        """ Determines whether the light is off.
        return: Boolean
        """
        return not self._sim_api.getLightParameters(self._handle)[0]

    # Get and Set Color

    def get_diffuse(self):
        """ Get the diffuse colors of the light.
        return: 3-vector np.array of diffuse colors
        """
        return np.asarray(self._sim_api.getLightParameters(self._handle)[2])

    def set_diffuse(self, diffuse):
        """ Set the diffuse colors of the light.
        """
        self._sim_api.setLightParameters(self._handle, self.is_on(), None, list(diffuse), None)

    def get_specular(self):
        """ Get the specular colors of the light.
        return: 3-vector np.array of specular colors
        """
        return np.asarray(self._sim_api.getLightParameters(self._handle)[3])

    def set_specular(self, specular):
        """ Set the specular colors of the light.
        """
        self._sim_api.setLightParameters(self._handle, self.is_on(), None, list(self.get_diffuse()), list(specular))

    # Intensity Properties

    def get_intensity_properties(self):
        """ Gets light intensity properties.

        :return: The light intensity properties cast_shadows, spot_exponent, spot_cutoff, const_atten_factor,
        linear_atten_factor, quad_atten_factor
        """
        cast_shadows = self._sim_api.getObjectInt32Param(self._handle, simc.sim_lightintparam_pov_casts_shadows)
        spot_exponent = self._sim_api.getObjectFloatParam(self._handle, simc.sim_lightfloatparam_spot_exponent)
        spot_cutoff = self._sim_api.getObjectFloatParam(self._handle, simc.sim_lightfloatparam_spot_cutoff)
        const_atten_factor = self._sim_api.getObjectFloatParam(self._handle, simc.sim_lightfloatparam_const_attenuation)
        linear_atten_factor = self._sim_api.getObjectFloatParam(self._handle, simc.sim_lightfloatparam_lin_attenuation)
        quad_atten_factor = self._sim_api.getObjectFloatParam(self._handle, simc.sim_lightfloatparam_quad_attenuation)
        return bool(cast_shadows), spot_exponent, spot_cutoff, const_atten_factor, linear_atten_factor, quad_atten_factor

    def set_intensity_properties(self, cast_shadows=None, spot_exponent=None, spot_cutoff=None, const_atten_factor=None,
                                 linear_atten_factor=None, quad_atten_factor=None):
        """ Set light intensity properties.

        :param cast_shadows: POV-Ray light casts shadows
        :param spot_exponent: light spot exponent
        :param spot_cutoff: light spot cutoff
        :param const_atten_factor: light constant attenuation factor, currently not supported
        :param linear_atten_factor: light linear attenuation factor, currently not supported
        :param quad_atten_factor: light quadratic attenuation factor, currently not supported
        """
        if cast_shadows is not None:
            self._sim_api.setObjectInt32Parameter(
                self._handle, simc.sim_lightintparam_pov_casts_shadows, int(cast_shadows))
        if spot_exponent is not None:
            if spot_exponent % 1 != 0:
                warnings.warn('spot exponent must be an integer, rounding input of {} to {}'.format(
                    spot_exponent, round(spot_exponent)))
            self._sim_api.setObjectFloatParameter(
                self._handle, simc.sim_lightfloatparam_spot_exponent, float(round(spot_exponent)))
        if spot_cutoff is not None:
            spot_cutoff = float(spot_cutoff)
            if spot_cutoff > np.pi/2:
                warnings.warn('Tried to set spot_cutoff to {}, but the maximum allowed value is pi/2,'
                              'therefore setting to pi/2'.format(spot_cutoff))
            self._sim_api.setObjectFloatParameter(
                self._handle, simc.sim_lightfloatparam_spot_cutoff, float(spot_cutoff))
        if const_atten_factor is not None:
            raise Exception('CoppeliaSim currently does not support setting attenuation factors')
        if linear_atten_factor is not None:
            raise Exception('CoppeliaSim currently does not support setting attenuation factors')
        if quad_atten_factor is not None:
            raise Exception('CoppeliaSim currently does not support setting attenuation factors')


object_type_to_class[ObjectType.LIGHT] = Light
