import ctypes


def load():
    call("require", ("scriptClientBridge",))


def call(func, args):
    from pyrep.backend.lib import (
        simCreateStack,
        simCallScriptFunctionEx,
        simReleaseStack,
        sim_scripttype_sandboxscript,
    )
    import pyrep.backend.stack as stack

    stackHandle = simCreateStack()
    stack.write(stackHandle, args)
    s = sim_scripttype_sandboxscript
    f = ctypes.c_char_p(f"{func}@lua".encode("ascii"))
    r = simCallScriptFunctionEx(s, f, stackHandle)
    if r == -1:
        if False:
            what = f"simCallScriptFunctionEx({s}, {func!r}, {args!r})"
        else:
            what = "simCallScriptFunctionEx"
        raise Exception(f"{what} returned -1")
    ret = stack.read(stackHandle)
    simReleaseStack(stackHandle)
    if len(ret) == 1:
        return ret[0]
    elif len(ret) > 1:
        return ret


def getObject(name, _info=None):
    ret = type(name, (), {})
    if not _info:
        _info = call("scriptClientBridge.info", [name])
    for k, v in _info.items():
        if not isinstance(v, dict):
            raise ValueError("found nondict")
        if len(v) == 1 and "func" in v:
            if f"{name}.{k}" == "sim.getScriptFunctions":
                setattr(
                    ret,
                    k,
                    lambda scriptHandle: type(
                        "",
                        (object,),
                        {
                            "__getattr__": lambda _, func: lambda *args: call(
                                "sim.callScriptFunction", (func, scriptHandle) + args
                            )
                        },
                    )(),
                )
                continue
            setattr(ret, k, lambda *a, func=f"{name}.{k}": call(func, a))
        elif len(v) == 1 and "const" in v:
            setattr(ret, k, v["const"])
        else:
            setattr(ret, k, getObject(f"{name}.{k}", _info=v))
    return ret


def require(obj):
    call("scriptClientBridge.require", [obj])
    o = getObject(obj)
    return o
