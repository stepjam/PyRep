from threading import Lock
from typing import List, Tuple
from pyrep.backend import sim
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.objects.joint import Joint
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.force_sensor import ForceSensor
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.camera import Camera
from pyrep.objects.octree import Octree

step_lock = Lock()


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
    elif t == sim.sim_object_camera_type:
        return Camera(handle)
    elif t == sim.sim_object_octree_type:
        return Octree(handle)
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
