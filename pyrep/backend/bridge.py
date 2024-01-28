from ctypes import *

from pyrep.backend.lib import *
from pyrep.errors import CoppeliaSimError


def __f(f):
    return f + '@lua'

def load():
    stack = simCreateStack()
    simPushStringOntoStack(stack, c_char_p('scriptClientBridge'.encode('ascii')), 0)
    r = simCallScriptFunctionEx(8, c_char_p(__f('require').encode('ascii')), stack)
    simReleaseStack(stack)

def call(func, args):
    stack = simCreateStack()
    import cbor
    b = cbor.dumps({'func': func, 'args': args})
    simPushStringOntoStack(stack, c_char_p(b), len(b))
    r = simCallScriptFunctionEx(8, c_char_p(__f('scriptClientBridge.call').encode('ascii')), stack)
    sz = c_int()
    ptr = simGetStackStringValue(stack, byref(sz))
    o = cbor.loads(string_at(ptr, sz.value))
    simReleaseBuffer(ptr)
    simReleaseStack(stack)
    if o['success']:
        ret = o['result']
        if len(ret) == 1:
            return ret[0]
        if len(ret) > 1:
            return tuple(ret)
    else:
        raise CoppeliaSimError(o['error'])

def getObject(name, _info=None):
    ret = type(name, (), {})
    if not _info:
        _info = call('scriptClientBridge.info', [name])
    for k, v in _info.items():
        if not isinstance(v, dict):
            raise ValueError('found nondict')
        if len(v) == 1 and 'func' in v:
            setattr(ret, k, lambda *a, func=f'{name}.{k}': call(func, a))
        elif len(v) == 1 and 'const' in v:
            setattr(ret, k, v['const'])
        else:
            setattr(ret, k, getObject(f'{name}.{k}', _info=v))
    return ret

def require(obj):
    call('scriptClientBridge.require', [obj])
    o = getObject(obj)
    return o
