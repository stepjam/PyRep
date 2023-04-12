"""CoppeliaSim's Remote API client."""

import os

import uuid

from time import sleep

import cbor

import zmq

import math

def b64(b):
    import base64
    return base64.b64encode(b).decode('ascii')


class RemoteAPIClient:
    """Client to connect to CoppeliaSim's ZMQ Remote API."""

    def __init__(self, host='localhost', port=23000, cntport=None, *, verbose=None):
        """Create client and connect to the ZMQ Remote API server."""
        self.verbose = int(os.environ.get('VERBOSE', '0')) if verbose is None else verbose
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.cntsocket = self.context.socket(zmq.SUB)
        self.socket.connect(f'tcp://{host}:{port}')
        self.cntsocket.setsockopt(zmq.SUBSCRIBE, b'')
        self.cntsocket.setsockopt(zmq.CONFLATE, 1)
        self.cntsocket.connect(f'tcp://{host}:{cntport if cntport else port+1}')
        self.uuid = str(uuid.uuid4())
        self.threadLocLevel = 0

    def __del__(self):
        """Disconnect and destroy client."""
        self.socket.close()
        self.cntsocket.close()
        self.context.term()

    def _send(self, req):
        if self.verbose > 0:
            print('Sending:', req)
        rawReq = cbor.dumps(req)
        if self.verbose > 1:
            print(f'Sending raw len={len(rawReq)}, base64={b64(rawReq)}')
        self.socket.send(rawReq)

    def _recv(self):
        rawResp = self.socket.recv()
        if self.verbose > 1:
            print(f'Received raw len={len(rawResp)}, base64={b64(rawResp)}')
        resp = cbor.loads(rawResp)
        if self.verbose > 0:
            print('Received:', resp)
        return resp

    def _process_response(self, resp):
        if not resp.get('success', False):
            raise Exception(resp.get('error'))
        ret = resp['ret']
        if len(ret) == 1:
            return ret[0]
        if len(ret) > 1:
            return tuple(ret)

    def call(self, func, args):
        """Call function with specified arguments."""
        self._send({'func': func, 'args': args})
        return self._process_response(self._recv())

    def getObject(self, name, _info=None):
        """Retrieve remote object from server."""
        ret = type(name, (), {})
        if not _info:
            _info = self.call('zmqRemoteApi.info', [name])
        for k, v in _info.items():
            if not isinstance(v, dict):
                raise ValueError('found nondict')
            if len(v) == 1 and 'func' in v:
                setattr(ret, k, lambda *a, func=f'{name}.{k}': self.call(func, a))
            elif len(v) == 1 and 'const' in v:
                setattr(ret, k, v['const'])
            else:
                setattr(ret, k, self.getObject(f'{name}.{k}', _info=v))
        if name=="sim":
            ret.wait=self._wait
            ret.waitForSignal=self._waitForSignal
            ret.moveToConfig=self._moveToConfig
            ret.moveToPose=self._moveToPose
            self.sim=ret
        return ret

    def setStepping(self, enable=True):
        ret = None
        if self.threadLocLevel > 0:
            self.threadLocLevel = 0
            ret = self.call('setStepping', [False,self.uuid])
        if enable == True:
            self.threadLocLevel = 1
            ret = self.call('setStepping', [enable,self.uuid])
        return ret

    def step(self, *, wait=True):
        if self.threadLocLevel > 0:
            self.getStepCount(False)
            self.call('step', [self.uuid])
            self.getStepCount(wait)

    def getStepCount(self, wait):
        if self.threadLocLevel > 0:
            try:
                self.cntsocket.recv(0 if wait else zmq.NOBLOCK)
            except zmq.ZMQError:
                pass

    def _setThreadAutomaticSwitch(self, level):
        newLevel = self.threadLocLevel
        if isinstance(level,bool):
            if level == True:
                newLevel -= 1
                if newLevel < 0:
                    newLevel = 0
            if level == False:
                newLevel += 1
        else:
            if level >= 0:
                newLevel = level
        if newLevel != self.threadLocLevel:
            if newLevel == 0:
                self.setStepping(False)
            if newLevel == 1 and self.threadLocLevel == 0:
                self.setStepping(True)
            self.threadLocLevel = newLevel
        return newLevel
        
    def _wait(self, dt, simTime=True):
        lb=self._setThreadAutomaticSwitch(False)
        retVal = 0.0
        if simTime:
            st = self.sim.getSimulationTime()
            while (self.sim.getSimulationTime()-st < dt):
                self.step()
            retVal=self.sim.getSimulationTime()-st-dt
        else:
            st = self.sim.getSystemTimeInMs(-1)
            while (self.sim.getSystemTimeInMs(st) < dt*1000):
                self.step()
        self._setThreadAutomaticSwitch(lb)
        return retVal

    def _waitForSignal(self, sigName):
        lb=self._setThreadAutomaticSwitch(False)
        retVal = 0.0
        while True:
            retVal = self.sim.getInt32Signal(sigName)!=None or self.sim.getFloatSignal(sigName)!=None or self.sim.getDoubleSignal(sigName)!=None or self.sim.getStringSignal(sigName)!=None
            if retVal:
                break
            self.step()
        self._setThreadAutomaticSwitch(lb)
        return retVal

    def _moveToConfig(self, flags,currentPos,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos,targetVel,callback,auxData=None,cyclicJoints=None,timeStep=0):
        lb=self._setThreadAutomaticSwitch(False)

        currentPosVelAccel=[]
        maxVelAccelJerk=[]
        targetPosVel=[]
        sel=[]
        outPos=[]
        outVel=[]
        outAccel=[]
        for i in range(len(currentPos)):
            v=currentPos[i]
            currentPosVelAccel.append(v)
            outPos.append(v)
            maxVelAccelJerk.append(maxVel[i])
            w=targetPos[i]
            if cyclicJoints and cyclicJoints[i]:
                while w-v>=math.pi*2:
                    w=w-math.pi*2
                while w-v<0:
                    w=w+math.pi*2
                if w-v>math.pi:
                    w=w-math.pi*2
            targetPosVel.append(w)
            sel.append(1)
        for i in range(len(currentPos)):
            if currentVel:
                currentPosVelAccel.append(currentVel[i])
                outVel.append(currentVel[i])
            else:
                currentPosVelAccel.append(0)
                outVel.append(0)
            maxVelAccelJerk.append(maxAccel[i])
            if targetVel:
                targetPosVel.append(targetVel[i])
            else:
                targetPosVel.append(0)
        for i in range(len(currentPos)):
            if currentAccel:
                currentPosVelAccel.append(currentAccel[i])
                outAccel.append(currentAccel[i])
            else:
                currentPosVelAccel.append(0)
                outAccel.append(0)
            maxVelAccelJerk.append(maxJerk[i])

        if len(maxVel) > len(currentPos):
            for i in range(len(maxVel)-len(currentPos)):
                currentPosVelAccel.append(maxVel[len(currentPos)+i])
        if len(maxAccel) > len(currentPos):
            for i in range(len(maxAccel)-len(currentPos)):
                currentPosVelAccel.append(maxAccel[len(currentPos)+i])
                
        ruckigObject = self.sim.ruckigPos(len(currentPos),0.0001,flags,currentPosVelAccel,maxVelAccelJerk,sel,targetPosVel)
        result = 0
        timeLeft = 0
        while (result == 0):
            dt = timeStep
            if dt == 0:
                dt=self.sim.getSimulationTimeStep()
            syncTime = 0
            result,newPosVelAccel,syncTime = self.sim.ruckigStep(ruckigObject,dt)
            if result >= 0:
                if result == 0:
                    timeLeft = dt-syncTime
                for i in range(len(currentPos)):
                    outPos[i]=newPosVelAccel[i]
                    outVel[i]=newPosVelAccel[len(currentPos)+i]
                    outAccel[i]=newPosVelAccel[len(currentPos)*2+i]
                if callback(outPos,outVel,outAccel,auxData):
                    break
            else:
                raise RuntimeError("sim.ruckigStep returned error code "+result)
            if result == 0:
                self.step()
        self.sim.ruckigRemove(ruckigObject)
        self._setThreadAutomaticSwitch(lb)
        return outPos,outVel,outAccel,timeLeft

    def _moveToPose(self, flags,currentPoseOrMatrix,maxVel,maxAccel,maxJerk,targetPoseOrMatrix,callback,auxData=None,metric=None,timeStep=0):
        lb = self._setThreadAutomaticSwitch(False)

        usingMatrices = (len(currentPoseOrMatrix)>=12)
        if usingMatrices:
            currentMatrix = currentPoseOrMatrix
            targetMatrix = targetPoseOrMatrix
        else:
            currentMatrix = self.sim.buildMatrixQ(currentPoseOrMatrix,[currentPoseOrMatrix[3],currentPoseOrMatrix[4],currentPoseOrMatrix[5],currentPoseOrMatrix[6]])
            targetMatrix = self.sim.buildMatrixQ(targetPoseOrMatrix,[targetPoseOrMatrix[3],targetPoseOrMatrix[4],targetPoseOrMatrix[5],targetPoseOrMatrix[6]])

        outMatrix = self.sim.copyTable(currentMatrix)
        axis,angle = self.sim.getRotationAxis(currentMatrix,targetMatrix)
        timeLeft = 0
        if metric:
            # Here we treat the movement as a 1 DoF movement, where we simply interpolate via t between
            # the start and goal pose. This always results in straight line movement paths
            dx = [(targetMatrix[3]-currentMatrix[3])*metric[0],(targetMatrix[7]-currentMatrix[7])*metric[1],(targetMatrix[11]-currentMatrix[11])*metric[2],angle*metric[3]]
            distance = math.sqrt(dx[0]*dx[0]+dx[1]*dx[1]+dx[2]*dx[2]+dx[3]*dx[3])
            if distance > 0.000001:
                currentPosVelAccel = [0,0,0]
                maxVelAccelJerk = [maxVel[0],maxAccel[0],maxJerk[0]]
                if len(maxVel) > 1:
                    maxVelAccelJerk.append(maxVel[1])
                if len(maxAccel) > 1:
                    maxVelAccelJerk.append(maxAccel[1])
                targetPosVel = [distance,0]
                ruckigObject = self.sim.ruckigPos(1,0.0001,flags,currentPosVelAccel,maxVelAccelJerk,[1],targetPosVel)
                result = 0
                while (result == 0):
                    dt = timeStep
                    if dt == 0:
                        dt = self.sim.getSimulationTimeStep()
                    result,newPosVelAccel,syncTime = self.sim.ruckigStep(ruckigObject,dt)
                    if result >= 0:
                        if result == 0:
                            timeLeft = dt-syncTime
                        t = newPosVelAccel[0]/distance
                        outMatrix = self.sim.interpolateMatrices(currentMatrix,targetMatrix,t)
                        nv = [newPosVelAccel[1]]
                        na = [newPosVelAccel[2]]
                        if not usingMatrices:
                            q = self.sim.getQuaternionFromMatrix(outMatrix)
                            outMatrix = [outMatrix[3],outMatrix[7],outMatrix[11],q[0],q[1],q[2],q[3]]
                        if callback(outMatrix,nv,na,auxData):
                            break
                    else:
                        raise RuntimeError("sim.ruckigStep returned error code "+result)
                    if result == 0:
                        self.step()
                self.sim.ruckigRemove(ruckigObject)
        else:
            # Here we treat the movement as a 4 DoF movement, where each of X, Y, Z and rotation
            # is handled and controlled individually. This can result in non-straight line movement paths,
            # due to how the Ruckig functions operate depending on 'flags'
            dx = [targetMatrix[3]-currentMatrix[3],targetMatrix[7]-currentMatrix[7],targetMatrix[11]-currentMatrix[11],angle]
            currentPosVelAccel = [0,0,0,0,0,0,0,0,0,0,0,0]
            maxVelAccelJerk = [maxVel[0],maxVel[1],maxVel[2],maxVel[3],maxAccel[0],maxAccel[1],maxAccel[2],maxAccel[3],maxJerk[0],maxJerk[1],maxJerk[2],maxJerk[3]]
            if len(maxVel) > 4:
                for i in range(len(maxVel)-len(maxJerk)):
                    maxVelAccelJerk.append(maxVel[len(maxJerk)+i])
            if len(maxAccel) > 4:
                for i in range(len(maxAccel)-len(maxJerk)):
                    maxVelAccelJerk.append(maxAccel[len(maxJerk)+i])
            targetPosVel = [dx[0],dx[1],dx[2],dx[3],0,0,0,0,0]
            ruckigObject = self.sim.ruckigPos(4,0.0001,flags,currentPosVelAccel,maxVelAccelJerk,[1,1,1,1],targetPosVel)
            result = 0
            while (result == 0):
                dt = timeStep
                if dt == 0:
                    dt = self.sim.getSimulationTimeStep()
                result,newPosVelAccel,syncTime = self.sim.ruckigStep(ruckigObject,dt)
                if result >= 0:
                    if result == 0:
                        timeLeft = dt-syncTime
                    t = 0
                    if abs(angle)>math.pi*0.00001:
                        t = newPosVelAccel[3]/angle
                    outMatrix = self.sim.interpolateMatrices(currentMatrix,targetMatrix,t)
                    outMatrix[3] = currentMatrix[3]+newPosVelAccel[0]
                    outMatrix[7] = currentMatrix[7]+newPosVelAccel[1]
                    outMatrix[11] = currentMatrix[11]+newPosVelAccel[2]
                    nv = [newPosVelAccel[4],newPosVelAccel[5],newPosVelAccel[6],newPosVelAccel[7]]
                    na = [newPosVelAccel[8],newPosVelAccel[9],newPosVelAccel[10],newPosVelAccel[11]]
                    if not usingMatrices:
                        q = self.sim.getQuaternionFromMatrix(outMatrix)
                        outMatrix = [outMatrix[3],outMatrix[7],outMatrix[11],q[0],q[1],q[2],q[3]]
                    if callback(outMatrix,nv,na,auxData):
                        break
                else:
                    raise RuntimeError("sim.ruckigStep returned error code "+result)
                if result == 0:
                    self.step()
            self.sim.ruckigRemove(ruckigObject)

        self._setThreadAutomaticSwitch(lb)
        return outMatrix,timeLeft


if __name__ == '__console__':
    client = RemoteAPIClient()
    sim = client.getObject('sim')


__all__ = ['RemoteAPIClient']
