from pyrep.backend import sim
from pyrep.const import ObjectType
from pyrep.objects.object import Object


class Gyroscope(Object):
    """An object able to measure angular velocities that are applied to it.
    """
    def __init__(self, name):
        super().__init__(name)
        self._ref = '%s_reference' % (self.get_name())

        self._last_time = sim.simGetSimulationTime()
        self._old_transformation_matrix = self.get_matrix()[:3, :4].reshape(
            (12, )).tolist()

    def _get_requested_type(self) -> ObjectType:
        return ObjectType(sim.simGetObjectType(self.get_handle()))

    def read(self):
        """Reads the angular velocities applied to gyroscope.

        :return: A list containing applied angular velocities along
            the sensor's x, y and z-axes.
        """
        current_time = sim.simGetSimulationTime()
        dt = current_time - self._last_time

        inv_old_matrix = sim.simInvertMatrix(self._old_transformation_matrix)
        transformation_matrix = self.get_matrix()[:3, :4].reshape(
            (12, )).tolist()
        mat = sim.simMultiplyMatrices(inv_old_matrix, transformation_matrix)
        euler_angles = sim.simGetEulerAnglesFromMatrix(mat)

        self._last_time = current_time
        self._old_transformation_matrix = transformation_matrix

        if dt != 0:
            return [euler_angle / dt for euler_angle in euler_angles]
        return [0.0, 0.0, 0.0]
