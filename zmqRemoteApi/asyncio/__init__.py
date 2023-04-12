"""CoppeliaSim's Remote API client."""

import asyncio
import sys
import os
import uuid

from contextlib import contextmanager

import cbor

import zmq
import zmq.asyncio


if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    if isinstance(asyncio.get_event_loop_policy(), asyncio.windows_events.WindowsProactorEventLoopPolicy):
        print("""

    WARNING: on Windows and Python 3.8+, `asyncio` might not work properly; in case, add the following before `asyncio.run(...)`:

    if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

""")


def b64(b):
    import base64
    return base64.b64encode(b).decode('ascii')


class RemoteAPIClient:
    """Client to connect to CoppeliaSim's ZMQ Remote API."""

    def __init__(self, host='localhost', port=23000, cntport=None, *, verbose=None):
        """Create client and connect to the ZMQ Remote API server."""
        self.verbose = int(os.environ.get('VERBOSE', '0')) if verbose is None else verbose
        self.host, self.port, self.cntport = host, port, cntport or port + 1
        self.cntsocket = None
        self.uuid=str(uuid.uuid4())
        # multiple sockets will be created for multiple concurrent requests, as needed
        self.sockets = []

    async def __aenter__(self):
        """Add one socket to the pool."""
        self.context = zmq.asyncio.Context()
        self.cntsocket = self.context.socket(zmq.SUB)
        self.cntsocket.setsockopt(zmq.SUBSCRIBE, b'')
        self.cntsocket.setsockopt(zmq.CONFLATE, 1)
        self.cntsocket.connect(f'tcp://{self.host}:{self.cntport}')
        return self

    async def __aexit__(self, *excinfo):
        """Disconnect and destroy client."""
        for socket in self.sockets:
            socket.close()
        self.cntsocket.close()
        self.context.term()

    @contextmanager
    def _socket(self):
        if not self.sockets:
            socket = self.context.socket(zmq.REQ)
            socket.connect(f'tcp://{self.host}:{self.port}')
            if self.verbose > 0:
                print('Added a new socket:', socket)
        else:
            socket = self.sockets.pop()
            if self.verbose > 0:
                print('Reusing existing socket:', socket)
        try:
            yield socket
        finally:
            self.sockets.append(socket)

    async def _send(self, socket, req):
        if self.verbose > 0:
            print('Sending:', req, socket)
        rawReq = cbor.dumps(req)
        if self.verbose > 1:
            print(f'Sending raw len={len(rawReq)}, base64={b64(rawReq)}')
        await socket.send(rawReq)

    async def _recv(self, socket):
        rawResp = await socket.recv()
        if self.verbose > 1:
            print(f'Received raw len={len(rawResp)}, base64={b64(rawResp)}')
        resp = cbor.loads(rawResp)
        if self.verbose > 0:
            print('Received:', resp, socket)
        return resp

    def _process_response(self, resp):
        if not resp.get('success', False):
            raise Exception(resp.get('error'))
        ret = resp['ret']
        if len(ret) == 1:
            return ret[0]
        if len(ret) > 1:
            return tuple(ret)

    async def call(self, func, args):
        """Call function with specified arguments."""
        with self._socket() as socket:
            await self._send(socket, {'func': func, 'args': args})
            return self._process_response(await self._recv(socket))

    async def getObject(self, name, _info=None):
        """Retrieve remote object from server."""
        ret = type(name, (), {})
        if not _info:
            _info = await self.call('zmqRemoteApi.info', [name])
        for k, v in _info.items():
            if not isinstance(v, dict):
                raise ValueError('found nondict')
            if len(v) == 1 and 'func' in v:
                setattr(ret, k, lambda *a, func=f'{name}.{k}': self.call(func, a))
            elif len(v) == 1 and 'const' in v:
                setattr(ret, k, v['const'])
            else:
                setattr(ret, k, self.getObject(f'{name}.{k}', _info=v))
        return ret

    async def setStepping(self, enable=True):
        return await self.call('setStepping', [enable,self.uuid])

    async def step(self, *, wait=True):
        await self.getStepCount(False)
        await self.call('step', [self.uuid])
        await self.getStepCount(wait)

    async def getStepCount(self, wait):
        try:
            await self.cntsocket.recv(0 if wait else zmq.NOBLOCK)
        except zmq.ZMQError:
            pass


__all__ = ['RemoteAPIClient']
