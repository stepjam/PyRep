import ctypes


def read_null(stackHandle):
    from pyrep.backend.lib import (
        simGetStackItemType,
        simPopStackItem,
        sim_stackitem_null,
    )

    if simGetStackItemType(stackHandle, -1) == sim_stackitem_null:
        simPopStackItem(stackHandle, 1)
        return None
    else:
        raise RuntimeError("expected nil")


def read_bool(stackHandle):
    from pyrep.backend.lib import (
        simGetStackBoolValue,
        simPopStackItem,
    )

    value = ctypes.c_bool()
    if simGetStackBoolValue(stackHandle, ctypes.byref(value)) == 1:
        simPopStackItem(stackHandle, 1)
        return value.value
    else:
        raise RuntimeError("expected bool")


def read_int(stackHandle):
    from pyrep.backend.lib import (
        simGetStackInt32Value,
        simPopStackItem,
    )

    value = ctypes.c_int()
    if simGetStackInt32Value(stackHandle, ctypes.byref(value)) == 1:
        simPopStackItem(stackHandle, 1)
        return value.value
    else:
        raise RuntimeError("expected int")


def read_long(stackHandle):
    from pyrep.backend.lib import (
        simGetStackInt64Value,
        simPopStackItem,
    )

    value = ctypes.c_longlong()
    if simGetStackInt64Value(stackHandle, ctypes.byref(value)) == 1:
        simPopStackItem(stackHandle, 1)
        return value.value
    else:
        raise RuntimeError("expected int64")


def read_double(stackHandle):
    from pyrep.backend.lib import (
        simGetStackDoubleValue,
        simPopStackItem,
    )

    value = ctypes.c_double()
    if simGetStackDoubleValue(stackHandle, ctypes.byref(value)) == 1:
        simPopStackItem(stackHandle, 1)
        return value.value
    else:
        raise RuntimeError("expected double")


def read_string(stackHandle, encoding=None):
    from pyrep.backend.lib import (
        simGetStackStringValue,
        simReleaseBuffer,
        simPopStackItem,
    )

    string_size = ctypes.c_int()
    string_ptr = simGetStackStringValue(stackHandle, ctypes.byref(string_size))
    value = ctypes.string_at(string_ptr, string_size.value)
    simPopStackItem(stackHandle, 1)
    if encoding:
        try:
            value = value.decode(encoding)
        except UnicodeDecodeError:
            pass
    simReleaseBuffer(string_ptr)
    return value


def read_dict(stackHandle):
    from pyrep.backend.lib import (
        simGetStackTableInfo,
        simGetStackSize,
        simUnfoldStackTable,
        simMoveStackItemToTop,
        sim_stack_table_map,
        sim_stack_table_empty,
    )

    d = dict()
    info = simGetStackTableInfo(stackHandle, 0)
    if info != sim_stack_table_map and info != sim_stack_table_empty:
        raise RuntimeError("expected a map")
    oldsz = simGetStackSize(stackHandle)
    simUnfoldStackTable(stackHandle)
    n = (simGetStackSize(stackHandle) - oldsz + 1) // 2
    for i in range(n):
        simMoveStackItemToTop(stackHandle, oldsz - 1)
        k = read_value(stackHandle)
        simMoveStackItemToTop(stackHandle, oldsz - 1)
        d[k] = read_value(stackHandle)
    return d


def read_list(stackHandle):
    from pyrep.backend.lib import (
        simGetStackSize,
        simUnfoldStackTable,
        simMoveStackItemToTop,
    )

    lst = list()
    oldsz = simGetStackSize(stackHandle)
    simUnfoldStackTable(stackHandle)
    n = (simGetStackSize(stackHandle) - oldsz + 1) // 2
    for i in range(n):
        simMoveStackItemToTop(stackHandle, oldsz - 1)
        read_value(stackHandle)
        simMoveStackItemToTop(stackHandle, oldsz - 1)
        lst.append(read_value(stackHandle))
    return lst


def read_table(stackHandle, typeHint=None):
    from pyrep.backend.lib import (
        simGetStackTableInfo,
        sim_stack_table_map,
        sim_stack_table_empty,
    )

    sz = simGetStackTableInfo(stackHandle, 0)
    if typeHint == "list" or sz >= 0:
        return read_list(stackHandle)
    elif typeHint == "dict" or sz in (sim_stack_table_map, sim_stack_table_empty):
        return read_dict(stackHandle)


def read_value(stackHandle, typeHint=None):
    from pyrep.backend.lib import (
        simGetStackItemType,
        sim_stackitem_null,
        sim_stackitem_double,
        sim_stackitem_bool,
        sim_stackitem_string,
        sim_stackitem_table,
        sim_stackitem_integer,
    )

    match typeHint:
        case "null":
            return read_null(stackHandle)
        case "float" | "double":
            return read_double(stackHandle)
        case "bool":
            return read_bool(stackHandle)
        case "string":
            return read_string(stackHandle, encoding="utf-8")
        case "buffer":
            return read_string(stackHandle, encoding=None)
        case "table" | "list" | "dict":
            return read_table(stackHandle, typeHint)
        case "int" | "long":
            return read_long(stackHandle)
    itemType = simGetStackItemType(stackHandle, -1)
    if itemType == sim_stackitem_null:
        return read_null(stackHandle)
    if itemType == sim_stackitem_double:
        return read_double(stackHandle)
    if itemType == sim_stackitem_bool:
        return read_bool(stackHandle)
    if itemType == sim_stackitem_string:
        return read_string(stackHandle, encoding="utf-8")
    if itemType == sim_stackitem_table:
        return read_table(stackHandle, typeHint)
    if itemType == sim_stackitem_integer:
        return read_long(stackHandle)
    raise RuntimeError(f"unexpected stack item type: {itemType} ({typeHint=})")


def read(stackHandle, typeHints=None):
    from pyrep.backend.lib import (
        simGetStackSize,
        simMoveStackItemToTop,
        simPopStackItem,
    )

    stack_size = simGetStackSize(stackHandle)
    tuple_data = []
    for i in range(stack_size):
        simMoveStackItemToTop(stackHandle, 0)
        if typeHints and len(typeHints) > i:
            value = read_value(stackHandle, typeHints[i])
        else:
            value = read_value(stackHandle)
        tuple_data.append(value)
    simPopStackItem(stackHandle, 0)  # clear all
    return tuple(tuple_data)


def write_null(stackHandle, value):
    from pyrep.backend.lib import (
        simPushNullOntoStack,
    )

    simPushNullOntoStack(stackHandle)


def write_double(stackHandle, value):
    from pyrep.backend.lib import (
        simPushDoubleOntoStack,
    )

    simPushDoubleOntoStack(stackHandle, value)


def write_bool(stackHandle, value):
    from pyrep.backend.lib import (
        simPushBoolOntoStack,
    )

    simPushBoolOntoStack(stackHandle, value)


def write_int(stackHandle, value):
    from pyrep.backend.lib import (
        simPushInt32OntoStack,
    )

    simPushInt32OntoStack(stackHandle, value)


def write_long(stackHandle, value):
    from pyrep.backend.lib import (
        simPushInt64OntoStack,
    )

    simPushInt64OntoStack(stackHandle, value)


def write_string(stackHandle, value, encoding="utf-8"):
    from pyrep.backend.lib import (
        simPushStringOntoStack,
    )

    if encoding:
        value = value.encode(encoding)
    simPushStringOntoStack(stackHandle, value, len(value))


def write_dict(stackHandle, value):
    from pyrep.backend.lib import (
        simPushTableOntoStack,
        simInsertDataIntoStackTable,
    )

    simPushTableOntoStack(stackHandle)
    for k, v in value.items():
        write_value(stackHandle, k)
        write_value(stackHandle, v)
        simInsertDataIntoStackTable(stackHandle)


def write_list(stackHandle, value):
    from pyrep.backend.lib import (
        simPushTableOntoStack,
        simInsertDataIntoStackTable,
    )

    simPushTableOntoStack(stackHandle)
    for i, v in enumerate(value):
        write_value(stackHandle, i + 1)
        write_value(stackHandle, v)
        simInsertDataIntoStackTable(stackHandle)


def write_value(stackHandle, value, typeHint=None):
    match typeHint:
        case "null":
            return write_null(stackHandle, value)
        case "float" | "double":
            return write_double(stackHandle, value)
        case "bool":
            return write_bool(stackHandle, value)
        case "int" | "long":
            return write_long(stackHandle, value)
        case "buffer":
            return write_string(stackHandle, value, encoding=None)
        case "string":
            return write_string(stackHandle, value, encoding="utf-8")
        case "dict":
            return write_dict(stackHandle, value)
        case "list":
            return write_list(stackHandle, value)
    if value is None:
        return write_null(stackHandle, value)
    elif isinstance(value, float):
        return write_double(stackHandle, value)
    elif isinstance(value, bool):
        return write_bool(stackHandle, value)
    elif isinstance(value, int):
        return write_long(stackHandle, value)
    elif isinstance(value, bytes):
        return write_string(stackHandle, value, encoding=None)
    elif isinstance(value, str):
        return write_string(stackHandle, value, encoding="utf-8")
    elif isinstance(value, dict):
        return write_dict(stackHandle, value)
    elif isinstance(value, list):
        return write_list(stackHandle, value)
    raise RuntimeError(f"unexpected type: {type(value)} ({typeHint=})")


def write(stackHandle, tuple_data, typeHints=None):
    for i, value in enumerate(tuple_data):
        if typeHints and len(typeHints) > i:
            write_value(stackHandle, value, typeHints[i])
        else:
            write_value(stackHandle, value)


def debug(stackHandle, info=None):
    from pyrep.backend.lib import (
        simGetStackSize,
        simDebugStack,
    )

    info = "" if info is None else f" {info} "
    n = (70 - len(info)) // 2
    m = 70 - len(info) - n
    print("#" * n + info + "#" * m)
    for i in range(simGetStackSize(stackHandle)):
        simDebugStack(stackHandle, i)
    print("#" * 70)


def callback(f):
    def wrapper(stackHandle):
        from typing import get_args

        inTypes = tuple(
            [
                arg_type.__name__
                for arg, arg_type in f.__annotations__.items()
                if arg != "return"
            ]
        )

        if return_annotation := f.__annotations__.get("return"):
            origin = getattr(return_annotation, "__origin__", None)
            if origin in (tuple, list):  # Handling built-in tuple and list
                outTypes = tuple([t.__name__ for t in get_args(return_annotation)])
            elif origin:  # Handling other generic types like Tuple, List from typing
                outTypes = (origin.__name__,)
            else:
                outTypes = (return_annotation.__name__,)
        else:
            outTypes = ()

        try:
            inArgs = read(stackHandle, inTypes)
            outArgs = f(*inArgs)
            if outArgs is None:
                outArgs = ()
            elif not isinstance(outArgs, tuple):
                outArgs = (outArgs,)
            write(stackHandle, outArgs, outTypes)
            return 1
        except Exception:
            import traceback

            traceback.print_exc()
            return 0

    return wrapper
