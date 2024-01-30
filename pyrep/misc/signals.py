from pyrep.backend.sim import SimBackend
from pyrep.errors import PyRepError
from pyrep.backend import sim
from typing import Any


class Signal(object):
    """Signals can be seen as global variables.

    Four types of signals are currently supported:
    integer-, floating-, double-, and string-type signals.
    Signals can be defined, redefined, read and cleared.
    """

    def __init__(self, name):
        self._name = name
        self._sim_api = SimBackend().sim_api

    def set(self, value) -> None:
        """Sets the value of this signal.

        :param value: The value of the signal.
        """
        pass

    def get(self) -> Any:
        """Gets the value of this signal.

        :raises PyRepError if signal does not exist.
        :return: The value of the signal.
        """
        pass

    def clear(self) -> int:
        """Clears the value of this signal.

        :return: The number of signals cleared. Either 0 or 1.
        """
        pass

    def _check_signal(self, value: int, type_name: str) -> None:
        if value == 0:
            raise PyRepError('Signal %s of type %s does not exist.' % (
                self._name, type_name))


class IntegerSignal(Signal):
    """An integer-type signal."""

    def set(self, value) -> None:
        self._sim_api.setInt32Signal(self._name, value)

    def get(self) -> int:
        return self._sim_api.getInt32Signal(self._name)

    def clear(self) -> None:
        self._sim_api.clearInt32Signal(self._name)


class FloatSignal(Signal):
    """A float-type signal."""

    def set(self, value) -> None:
        self._sim_api.setFloatSignal(self._name, value)

    def get(self) -> float:
        return self._sim_api.getFloatSignal(self._name)

    def clear(self) -> None:
        self._sim_api.clearFloatSignal(self._name)


class StringSignal(Signal):
    """A string-type signal."""

    def set(self, value) -> None:
        self._sim_api.setStringSignal(self._name, value)

    def get(self) -> str:
        return self._sim_api.getStringSignal(self._name)

    def clear(self) -> None:
        self._sim_api.clearStringSignal(self._name)
