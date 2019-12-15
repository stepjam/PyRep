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
        sim.simSetIntegerSignal(self._name, value)

    def get(self) -> int:
        ret, value = sim.simGetIntegerSignal(self._name)
        self._check_signal(ret, 'int')
        return value

    def clear(self) -> int:
        return sim.simClearIntegerSignal(self._name)


class FloatSignal(Signal):
    """An float-type signal."""

    def set(self, value) -> None:
        sim.simSetFloatSignal(self._name, value)

    def get(self) -> float:
        ret, value = sim.simGetFloatSignal(self._name)
        self._check_signal(ret, 'float')
        return value

    def clear(self) -> int:
        return sim.simClearFloatSignal(self._name)


class DoubleSignal(Signal):
    """An double-type signal."""

    def set(self, value) -> None:
        sim.simSetDoubleSignal(self._name, value)

    def get(self) -> float:
        ret, value = sim.simGetDoubleSignal(self._name)
        self._check_signal(ret, 'double')
        return value

    def clear(self) -> int:
        return sim.simClearDoubleSignal(self._name)


class StringSignal(Signal):
    """An string-type signal."""

    def set(self, value) -> None:
        sim.simSetStringSignal(self._name, value)

    def get(self) -> str:
        ret, value = sim.simGetStringSignal(self._name)
        self._check_signal(ret, 'string')
        return value

    def clear(self) -> int:
        return sim.simClearStringSignal(self._name)
