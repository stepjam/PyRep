import builtins
import os
import sys

from ctypes import *

from pyrep.errors import PyRepError

if 'COPPELIASIM_ROOT' not in os.environ:
    raise PyRepError(
        'COPPELIASIM_ROOT not defined. See installation instructions.')
coppeliasim_root = os.environ['COPPELIASIM_ROOT']
coppeliasim_library = os.path.join(coppeliasim_root, "libcoppeliaSim.so")
if not os.path.isfile(coppeliasim_library):
    raise PyRepError(
        'COPPELIASIM_ROOT was not a correct path. '
        'See installation instructions')

appDir = os.path.dirname(coppeliasim_library)
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = appDir

import platform
plat = platform.system()
if plat == 'Darwin':
    fd = os.path.normpath(appDir + '/../Frameworks')
    os.environ['DYLD_LIBRARY_PATH'] = fd + ':' + os.environ.get('DYLD_LIBRARY_PATH', '')
    print(f'If next step fails, do: export DYLD_LIBRARY_PATH={fd}:$DYLD_LIBRARY_PATH and relaunch.')

coppeliaSimLib = cdll.LoadLibrary(coppeliasim_library)
coppeliaSimLib.simRunGui.argtypes = [c_int]
coppeliaSimLib.simRunGui.restype = None
coppeliaSimLib.simCreateStack.argtypes = []
coppeliaSimLib.simCreateStack.restype = c_int
coppeliaSimLib.simReleaseStack.argtypes = [c_int]
coppeliaSimLib.simReleaseStack.restype = c_int
coppeliaSimLib.simReleaseBuffer.argtypes = [c_void_p]
coppeliaSimLib.simReleaseBuffer.restype = c_int
coppeliaSimLib.simPushStringOntoStack.argtypes = [c_int, c_char_p, c_int]
coppeliaSimLib.simPushStringOntoStack.restype = c_int
coppeliaSimLib.simCallScriptFunctionEx.argtypes = [c_int, c_char_p, c_int]
coppeliaSimLib.simCallScriptFunctionEx.restype = c_int
coppeliaSimLib.simGetStackStringValue.argtypes = [c_int, POINTER(c_int)]
coppeliaSimLib.simGetStackStringValue.restype = c_void_p
coppeliaSimLib.simInitialize.argtypes = [c_char_p, c_int]
coppeliaSimLib.simInitialize.restype = c_int
coppeliaSimLib.simGetExitRequest.argtypes = []
coppeliaSimLib.simGetExitRequest.restype = c_int
coppeliaSimLib.simLoop.argtypes = [c_void_p, c_int]
coppeliaSimLib.simLoop.restype = c_int
coppeliaSimLib.simDeinitialize.argtypes = []
coppeliaSimLib.simDeinitialize.restype = c_int
coppeliaSimLib.simSetStringParam.argtypes = [c_int, c_char_p]
coppeliaSimLib.simSetStringParam.restype = c_int
coppeliaSimLib.simSetNamedStringParam.argtypes = [c_char_p, c_char_p, c_int]
coppeliaSimLib.simSetNamedStringParam.restype = c_int
coppeliaSimLib.simRegCallback.argtypes = [c_int, CFUNCTYPE(c_int, c_int)]
coppeliaSimLib.simRegCallback.restype = None
coppeliaSimLib.simCopyStack.argtypes = [c_int]
coppeliaSimLib.simCopyStack.restype = c_int
coppeliaSimLib.simPushNullOntoStack.argtypes = [c_int]
coppeliaSimLib.simPushNullOntoStack.restype = c_int
coppeliaSimLib.simPushBoolOntoStack.argtypes = [c_int, c_bool]
coppeliaSimLib.simPushBoolOntoStack.restype = c_int
coppeliaSimLib.simPushInt32OntoStack.argtypes = [c_int, c_int]
coppeliaSimLib.simPushInt32OntoStack.restype = c_int
coppeliaSimLib.simPushInt64OntoStack.argtypes = [c_int, c_longlong]
coppeliaSimLib.simPushInt64OntoStack.restype = c_int
coppeliaSimLib.simPushUInt8TableOntoStack.argtypes = [c_int, POINTER(c_ubyte), c_int]
coppeliaSimLib.simPushUInt8TableOntoStack.restype = c_int
coppeliaSimLib.simPushInt32TableOntoStack.argtypes = [c_int, POINTER(c_int), c_int]
coppeliaSimLib.simPushInt32TableOntoStack.restype = c_int
coppeliaSimLib.simPushInt64TableOntoStack.argtypes = [c_int, POINTER(c_longlong), c_int]
coppeliaSimLib.simPushInt64TableOntoStack.restype = c_int
coppeliaSimLib.simPushTableOntoStack.argtypes = [c_int]
coppeliaSimLib.simPushTableOntoStack.restype = c_int
coppeliaSimLib.simInsertDataIntoStackTable.argtypes = [c_int]
coppeliaSimLib.simInsertDataIntoStackTable.restype = c_int
coppeliaSimLib.simGetStackSize.argtypes = [c_int]
coppeliaSimLib.simGetStackSize.restype = c_int
coppeliaSimLib.simPopStackItem.argtypes = [c_int, c_int]
coppeliaSimLib.simPopStackItem.restype = c_int
coppeliaSimLib.simMoveStackItemToTop.argtypes = [c_int, c_int]
coppeliaSimLib.simMoveStackItemToTop.restype = c_int
coppeliaSimLib.simGetStackItemType.argtypes = [c_int, c_int]
coppeliaSimLib.simGetStackItemType.restype = c_int
coppeliaSimLib.simGetStackBoolValue.argtypes = [c_int, POINTER(c_bool)]
coppeliaSimLib.simGetStackBoolValue.restype = c_int
coppeliaSimLib.simGetStackInt32Value.argtypes = [c_int, POINTER(c_int)]
coppeliaSimLib.simGetStackInt32Value.restype = c_int
coppeliaSimLib.simGetStackInt64Value.argtypes = [c_int, POINTER(c_longlong)]
coppeliaSimLib.simGetStackInt64Value.restype = c_int
coppeliaSimLib.simGetStackTableInfo.argtypes = [c_int, c_int]
coppeliaSimLib.simGetStackTableInfo.restype = c_int
coppeliaSimLib.simGetStackUInt8Table.argtypes = [c_int, c_char_p, c_int]
coppeliaSimLib.simGetStackUInt8Table.restype = c_int
coppeliaSimLib.simGetStackInt32Table.argtypes = [c_int, POINTER(c_int), c_int]
coppeliaSimLib.simGetStackInt32Table.restype = c_int
coppeliaSimLib.simGetStackInt64Table.argtypes = [c_int, POINTER(c_longlong), c_int]
coppeliaSimLib.simGetStackInt64Table.restype = c_int
coppeliaSimLib.simUnfoldStackTable.argtypes = [c_int]
coppeliaSimLib.simUnfoldStackTable.restype = c_int
coppeliaSimLib.simGetStackDoubleValue.argtypes = [c_int, POINTER(c_double)]
coppeliaSimLib.simGetStackDoubleValue.restype = c_int
coppeliaSimLib.simGetStackDoubleTable.argtypes = [c_int, POINTER(c_double), c_int]
coppeliaSimLib.simGetStackDoubleTable.restype = c_int
coppeliaSimLib.simPushDoubleOntoStack.argtypes = [c_int, c_double]
coppeliaSimLib.simPushDoubleOntoStack.restype = c_int
coppeliaSimLib.simPushDoubleTableOntoStack.argtypes = [c_int, POINTER(c_double), c_int]
coppeliaSimLib.simPushDoubleTableOntoStack.restype = c_int

__all__ = []

for name in dir(coppeliaSimLib):
    if name.startswith('sim'):
        f = getattr(coppeliaSimLib, name)
        if callable(f):
            globals()[name] = f
            __all__.append(name)

const = type('', (), {})

const.sim_stackitem_null = 0
const.sim_stackitem_double = 1
const.sim_stackitem_bool = 2
const.sim_stackitem_string = 3
const.sim_stackitem_table = 4
const.sim_stackitem_func = 5
const.sim_stackitem_userdat = 6
const.sim_stackitem_thread = 7
const.sim_stackitem_lightuserdat = 8
const.sim_stackitem_integer = 9

const.sim_stack_table_circular_ref = -4
const.sim_stack_table_not_table = -3
const.sim_stack_table_map = -2
const.sim_stack_table_empty = 0

for name in dir(const):
    if name.startswith('sim'):
        f = getattr(const, name)
        globals()[name] = f
        __all__.append(name)
