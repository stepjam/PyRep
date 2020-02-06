import os
import io
import sys
from contextlib import contextmanager
from typing import List, Tuple
import pyrep
from pyrep.backend import sim
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.objects.joint import Joint
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.force_sensor import ForceSensor
from pyrep.objects.proximity_sensor import ProximitySensor


def to_type(handle: int) -> Object:
    """Converts an object handle to the correct sub-type.

    :param handle: The internal handle of an object.
    :return: The sub-type of this object.
    """
    t = sim.simGetObjectType(handle)
    if t == sim.sim_object_shape_type:
        return Shape(handle)
    elif t == sim.sim_object_dummy_type:
        return Dummy(handle)
    elif t == sim.sim_object_path_type:
        return CartesianPath(handle)
    elif t == sim.sim_object_joint_type:
        return Joint(handle)
    elif t == sim.sim_object_visionsensor_type:
        return VisionSensor(handle)
    elif t == sim.sim_object_forcesensor_type:
        return ForceSensor(handle)
    elif t == sim.sim_object_proximitysensor_type:
        return ProximitySensor(handle)
    raise ValueError


def script_call(function_name_at_script_name: str,
                script_handle_or_type: int,
                ints=(), floats=(), strings=(), bytes='') -> (
        Tuple[List[int], List[float], List[str], str]):
    """Calls a script function (from a plugin, the main client application,
    or from another script). This represents a callback inside of a script.

    :param function_name_at_script_name: A string representing the function
        name and script name, e.g. myFunctionName@theScriptName. When the
        script is not associated with an object, then just specify the
        function name.
    :param script_handle_or_type: The handle of the script, otherwise the
        type of the script.
    :param ints: The input ints to the script.
    :param floats: The input floats to the script.
    :param strings: The input strings to the script.
    :param bytes: The input bytes to the script (as a string).
    :return: Any number of return values from the called Lua function.
    """
    return sim.simExtCallScriptFunction(
        function_name_at_script_name, script_handle_or_type, list(ints),
        list(floats), list(strings), bytes)


def _is_in_ipython():
    try:
        __IPYTHON__
        return True
    except NameError:
        pass
    return False


@contextmanager
def suppress_std_out_and_err():
    """Used for suppressing std out/err.

    This is needed because the OMPL plugin outputs logging info even when
    logging is turned off.
    """
    try:
        # If we are using an IDE, then this will fail
        original_stdout_fd = sys.stdout.fileno()
        original_stderr_fd = sys.stderr.fileno()
    except io.UnsupportedOperation:
        # Nothing we can do about this, just don't suppress
        yield
        return

    if _is_in_ipython():
        yield
        return

    with open(os.devnull, "w") as devnull:

        devnull_fd = devnull.fileno()

        def _redirect_stdout(to_fd):
            sys.stdout.close()
            os.dup2(to_fd, original_stdout_fd)
            if pyrep.testing:
                sys.stdout = io.TextIOWrapper(
                    os.fdopen(original_stdout_fd, 'wb'))
            else:
                sys.stdout = os.fdopen(original_stdout_fd, 'w')

        def _redirect_stderr(to_fd):
            sys.stderr.close()
            os.dup2(to_fd, original_stderr_fd)
            if pyrep.testing:
                sys.stderr = io.TextIOWrapper(
                    os.fdopen(original_stderr_fd, 'wb'))
            else:
                sys.stderr = os.fdopen(original_stderr_fd, 'wb')

        saved_stdout_fd = os.dup(original_stdout_fd)
        # saved_stderr_fd = os.dup(original_stderr_fd)

        try:
            _redirect_stdout(devnull_fd)
            # _redirect_stderr(devnull_fd)
            yield
            _redirect_stdout(saved_stdout_fd)
            # _redirect_stderr(saved_stderr_fd)
        finally:
            os.close(saved_stdout_fd)
            # os.close(saved_stderr_fd)
