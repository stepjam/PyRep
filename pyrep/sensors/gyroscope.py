from pyrep.backend import sim
from pyrep.const import ObjectType
from pyrep.objects.object import Object


class Gyroscope(Object):
    """An object able to measure angular velocities that are applied to it.
    """
    def __init__(self, name):
        super().__init__(name)
        self._ref = '%s_reference' % (self.get_name())

        self._last_time = self._sim_api.getSimulationTime()
        self._old_transformation_matrix = self.get_matrix()[:3, :4].reshape(
            (12, )).tolist()

    def _get_requested_type(self) -> ObjectType:
        return ObjectType(self._sim_api.getObjectType(self.get_handle()))

    def read(self):
        """Reads the angular velocities applied to gyroscope.

        :return: A list containing applied angular velocities along
            the sensor's x, y and z-axes.
        """
        current_time = self._sim_api.getSimulationTime()
        dt = current_time - self._last_time

        inv_old_matrix = self._sim_api.getMatrixInverse(self._old_transformation_matrix)
        transformation_matrix = self.get_matrix()[:3, :4].reshape(
            (12, )).tolist()
        mat = self._sim_api.multiplyMatrices(inv_old_matrix, transformation_matrix)
        euler_angles = self._sim_api.getEulerAnglesFromMatrix(mat)

        self._last_time = current_time
        self._old_transformation_matrix = transformation_matrix

        if dt != 0:
            return [euler_angle / dt for euler_angle in euler_angles]
        return [0.0, 0.0, 0.0]
