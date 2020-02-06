from pyrep.errors import PyRepError
from pyrep.backend import sim
from typing import Union


class Distance(object):
    """Allows registering distance objects which are measurable entity-pairs."""

    def __init__(self, name_or_handle: Union[str, int]):
        raise PyRepError(
            'Currently there is an error in CoppeliaSim with distance objects. '
            'As soon as CoppeliaSim resolves this issue, this error will be '
            'removed.')
        self._handle: int
        if isinstance(name_or_handle, int):
            self._handle = name_or_handle
        else:
            self._handle = sim.simGetDistanceHandle(name_or_handle)
        # The entity needs to be set as explicit handling for reads to work.
        if not sim.simGetExplicitHandling(self._handle):
            sim.simSetExplicitHandling(self._handle, 1)

    def __eq__(self, other: object):
        if not isinstance(other, Distance):
            raise NotImplementedError
        return self.get_handle() == other.get_handle()

    def get_handle(self) -> int:
        """Gets the internal handle of this object.

        :return: The internal handle.
        """
        return self._handle

    def read(self) -> Union[float, None]:
        """Reads the distance of a registered distance object.

        :raises: PyRepError if no objects could be measured.

        :return: The smallest distance between the 2 entities or None if no
            measurement could be made.
        """
        num_measurements = sim.simHandleDistance(self._handle)
        if num_measurements == 0:
            raise PyRepError(
                'Could not make a measurement. Are both entities associated '
                'with the distance object marked as measurable?')
        return (None if num_measurements == 0
                else sim.simReadDistance(self._handle))
