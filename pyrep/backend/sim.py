import time
import warnings

# from func_timeout import func_timeout
# import cffi

from .simConst import *
import numpy as np
import collections
from zmqRemoteApi import RemoteAPIClient

lib = None
client = RemoteAPIClient()
try:
    sim =  client.getObject('sim')
    simIK = client.getObject('simIK')
    lib=sim
    client.setStepping(True)
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.1)
    sim.startSimulation()
    client.step()
except:
    raise IOError("Failed to connect to Coppeliasim, either open it or set setup.use_coppelia_sim to false")


SShapeVizInfo = collections.namedtuple(
    'SShapeVizInfo',
    [
        'vertices',
        'indices',
        'normals',
        'shadingAngle',
        'colors',
        'texture',
        'textureId',
        'textureRes',
        'textureCoords',
        'textureApplyMode',
        'textureOptions',
    ],
)


def _check_return(ret):
    if ret < 0:
        raise RuntimeError(
            'The call failed on the V-REP side. Return value: %d' % ret)


def _check_null_return(val):
    if val == None:
        raise RuntimeError(
            'The call failed on the V-REP side by returning null.')


def _check_set_object_parameter(ret):
    if ret == 0:
        raise RuntimeError(
            'Parameter could not be set (e.g. because the parameterID doesn\'t '
            'exist, or because the specified object doesn\'t correspond to the '
            'correct type)')


def simExtLaunchUIThread(options, scene, pyrep_root):
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.1)
    sim.loadScene(scene )
    # lib.simExtLaunchUIThread(
    #     'PyRep'.encode('ascii'), options, scene.encode('ascii'),
    #     pyrep_root.encode('ascii'))
    client.setStepping(True)
    sim.startSimulation()
    client.step()


def simExtSimThreadInit():
    # lib.simExtSimThreadInit()
    pass

def simExtCanInitSimThread():

    # return bool(lib.simExtCanInitSimThread())
    return bool(True)


def simExtSimThreadDestroy():
    pass
    # lib.simExtSimThreadDestroy()


def simExtPostExitRequest():
    pass
    # lib.simExtPostExitRequest()


def simExtGetExitRequest():
    return bool(sim.extGetExitRequest())


def simExtStep(stepIfRunning=True):
    client.step()
    #lib.simExtStep(stepIfRunning)


def simStartSimulation():
    #lib.simStartSimulation()
    sim.startSimulation()


def simStopSimulation():
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.1)


def simPauseSimulation():
    return sim.pauseSimulation()


def simQuitSimulator(doNotDisplayMessages):
    pass
    # lib.simQuitSimulator(doNotDisplayMessages)


def simGetObjectHandle(objectName):
    # handle = sim.getObjectHandle(objectName.encode('ascii'))
    #add / to name
    handle = sim.getObjectHandle(objectName.encode('ascii'))
    if handle < 0:
        raise RuntimeError('Handle %s does not exist.' % objectName)
    return handle


def simGetIkGroupHandle(ikGroupName):
    handle = sim.getIkGroupHandle(ikGroupName.encode('ascii'))
    if handle <= 0:
        raise RuntimeError('Ik group does not exist.')
    return handle

def simGetIkAndEnv(ikGroupName):
    # this should be arm specific not in sim
    ik_target_handle = sim.getObject("/"+ikGroupName.replace("_ik","/IK"))
    scriptHandle = sim.getScript(sim.scripttype_childscript, ik_target_handle)
    ikEnv, ikGroup, ikJoints,ikBase, ikTip, ikTarget = sim.callScriptFunction('getEnvAndGroup', scriptHandle)
    return ikEnv, ikGroup, ikJoints, ikBase, ikTip, ikTarget

def simSetIkElementProperties(ikGroupHandle, tipDummyHandle, constraints,
                              precision=None, weight=None):
    if precision is None:
        precision = None
    if weight is None:
        weight = None
    reserved = None
    ret = sim.setIkElementProperties(
        ikGroupHandle, tipDummyHandle, constraints, precision, weight, reserved)
    _check_return(ret)
    return ret


def simSetIkGroupProperties(ikGroupHandle, resolutionMethod, maxIterations, damping):
    reserved = None
    ret = sim.setIkGroupProperties(
        ikGroupHandle, resolutionMethod, maxIterations, damping, reserved)
    _check_return(ret)
    return ret


def simGetObjectPosition(objectHandle, relativeToObjectHandle):

    position = sim.getObjectPosition(objectHandle, relativeToObjectHandle)
    return list(position)


def simGetJointPosition(jointHandle):
    return sim.getJointPosition(jointHandle)



def simSetJointPosition(jointHandle, position):
    sim.setJointPosition(jointHandle, position)


def simGetJointMatrix(jointHandle):
    return sim.getJointMatrix(jointHandle)


def simSetSphericalJointMatrix(jointHandle, matrix):
    sim.setSphericalJointMatrix(jointHandle, matrix)


def simGetJointTargetVelocity(jointHandle):
    return sim.getJointTargetVelocity(jointHandle)


def simSetJointTargetVelocity(jointHandle, targetVelocity):
    sim.setJointTargetVelocity(jointHandle, targetVelocity)


def simGetJointTargetPosition(jointHandle):
    return sim.getJointTargetPosition(jointHandle)


def simSetJointTargetPosition(jointHandle, targetPosition):
    sim.setJointTargetPosition(jointHandle, targetPosition)


def simGetJointForce(jointHandle):
    return sim.getJointForce(jointHandle)

def simSetJointForce(jointHandle, force):
    sim.setJointForce(jointHandle, force)


def simGetJointMaxForce(jointHandle):
    return sim.getJointMaxForce(jointHandle)


def simSetJointMaxForce(jointHandle, force):
    sim.setJointMaxForce(jointHandle, force)


def simGetJointInterval(jointHandle):
    cyclic,interval  = sim.getJointInterval(jointHandle)
    return cyclic, interval


def simSetJointInterval(jointHandle, cyclic, interval):
    sim.setJointInterval(jointHandle, cyclic, interval)



def simCreateForceSensor(options, intParams, floatParams, color):
    handle = sim.createForceSensor(options, intParams, floatParams, color)
    _check_return(handle)
    return handle


def simBreakForceSensor(forceSensorHandle):
    sim.breakForceSensor(forceSensorHandle)

def simReadForceSensor(forceSensorHandle):
    state, forceVector, torqueVector = sim.readForceSensor(forceSensorHandle)
    return state, forceVector ,  torqueVector


def simReleaseBuffer(pointer):
    warnings.warn('no buffers')
    pass
    # lib.simReleaseBuffer(pointer)


def simCreateVisionSensor(options, intParams, floatParams, color):

    ret = sim.createVisionSensor(options, intParams, floatParams, color)
    _check_return(ret)
    return ret


def simHandleVisionSensor(sensorHandle):
    count,auxValues,_ = sim.handleVisionSensor(
        sensorHandle )
    return 1, auxValues


def simReadVisionSensor(sensorHandle):
    res, auxValues,_ = sim.readVisionSensor(sensorHandle)
    return res, auxValues


def simGetVisionSensorImage(sensorHandle, resolution):
    img, resX, resY = sim.getVisionSensorCharImage(sensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
    img = img.reshape(resolution[1], resolution[0], 3)
    img = np.flip(img, 0).copy()  # Image is upside-down
    return img


def simGetVisionSensorDepthBuffer(sensorHandle, resolution, in_meters):

    img_buffer = sim.getVisionSensorDepth(sensorHandle,
    options = 1 if in_meters else 0,
    pos = [0, 0],
    size = [0, 0])
    img = np.frombuffer(img_buffer, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
    img = img.reshape(resolution[1], resolution[0], 3)
    img = np.flip(img, 0).copy()  # Image is upside-down
    return img


def simGetVisionSensorResolution(sensorHandle):
    return sim.getVisionSensorResolution(sensorHandle)


def simGetObjectChild(parentObjectHandle, childIndex):
    val = sim.getObjectChild(parentObjectHandle, childIndex)
    _check_return(val)
    return val


def simGetObjectParent(childObjectHandle):
    val = sim.getObjectParent(childObjectHandle)
    _check_return(val)
    return val


def simReadProximitySensor(sensorHandle):

    state, distance, detectedPoint, detectedObjectHandle, \
        detectedSurfaceNormalVector = \
        sim.readProximitySensor(sensorHandle)
    _check_return(state)
    return (state, detectedObjectHandle, list(detectedPoint),
            list(detectedSurfaceNormalVector))


def simCheckProximitySensor(sensorHandle, entityHandle):
    state, distance, detectedPoint, detectedObjectHandle,\
        surfaceNormalVector = \
        sim.checkProximitySensor(sensorHandle,entityHandle)
    return state, detectedPoint


def simLoadModel(modelPathAndName):
    val = sim.loadModel(modelPathAndName.encode('ascii'))
    _check_return(val)
    return val


def simLoadScene(scenePathAndName):
    val = sim.loadScene(scenePathAndName.encode('ascii'))
    _check_return(val)
    return val


def simSaveModel(modelHandle, modelPathAndName):
    val = sim.saveModel(modelHandle, modelPathAndName.encode('ascii'))
    _check_return(val)
    return val


def simSaveScene(filename):
    val = sim.saveScene(filename.encode('ascii'))
    _check_return(val)
    return val


def simGetObjectName(objectHandle):
    name = sim.getObjectName(objectHandle)
    return name


def simSetObjectName(objectHandle, name):
    ret = sim.setObjectName(objectHandle, name.encode('ascii'))
    _check_return(ret)


def simAddStatusbarMessage(message):
    return sim.addStatusbarMessage(message.encode('ascii'))


def simGetObjectOrientation(objectHandle, relativeToObjectHandle):
    eulerAngles = sim.getObjectOrientation(
        objectHandle, relativeToObjectHandle)
    return eulerAngles


def simGetObjectQuaternion(objectHandle, relativeToObjectHandle):
    return sim.getObjectQuaternion(
        objectHandle, relativeToObjectHandle)


def simSetObjectOrientation(objectHandle, relativeToObjectHandle, eulerAngles):
    ret = sim.setObjectOrientation(
        objectHandle, relativeToObjectHandle, eulerAngles)
    _check_return(ret)


def simSetObjectQuaternion(objectHandle, relativeToObjectHandle, quaternion):
    ret = sim.setObjectQuaternion(
        objectHandle, relativeToObjectHandle, quaternion)
    _check_return(ret)


def simSetObjectPosition(objectHandle, relativeToObjectHandle, position):
    ret = sim.setObjectPosition(
        objectHandle, relativeToObjectHandle, position)
    _check_return(ret)


def simSetObjectParent(objectHandle, parentObject, keepInPlace):
    ret = sim.setObjectParent(objectHandle, parentObject, keepInPlace)
    _check_return(ret)

def simGetArrayParameter(paramIdentifier):
    paramValues = sim.getArrayParam(paramIdentifier)
    return paramValues


def simSetArrayParameter(paramIdentifier, paramValues):
    ret = sim.setArrayParam(paramIdentifier, paramValues)
    _check_return(ret)


def simGetBoolParameter(parameter):
    ret = sim.getBoolParam(parameter)
    _check_return(ret)
    return ret


def simSetBoolParameter(parameter, value):
    ret = sim.setBoolParam(parameter, value)
    _check_return(ret)


def simGetInt32Parameter(parameter):
    ret = sim.getInt32Param(parameter)
    _check_return(ret)
    return ret


def simSetInt32Parameter(parameter, value):
    ret = sim.setInt32Param(parameter, value)
    _check_return(ret)


def simGetFloatParameter(parameter):
    ret = sim.getFloatParam(parameter)
    _check_return(ret)
    return ret


def simSetFloatParameter(parameter, value):
    ret = sim.setFloatParam(parameter, value)
    _check_return(ret)


def simSetStringParameter(parameter, value):
    ret = sim.setStringParam(parameter, value.encode('ascii'))
    # ret = sim.setStringParam(parameter, value.encode('ascii'))
    _check_return(ret)


def simGetStringParameter(parameter):
    return sim.getStringParam(parameter)



def simGetEngineFloatParameter(parameter, objectHandle):
    return sim.getEngineFloatParam(parameter, objectHandle)


def simSetEngineFloatParameter(parameter, objectHandle, val):
    sim.setEngineFloatParam(parameter, objectHandle,
                                         val)


def simGetCollisionHandle(collisionObjectName):
    ret = sim.getCollisionHandle(collisionObjectName.encode('ascii'))
    _check_return(ret)
    return ret


def simGetCollectionHandle(collectionName):
    ret = sim.getCollectionHandle(collectionName.encode('ascii'))
    _check_return(ret)
    return ret


def simGetDistanceHandle(distanceObjectName):
    ret = sim.getDistanceHandle(distanceObjectName.encode('ascii'))
    _check_return(ret)
    return ret


def simReadCollision(collisionObjectHandle):
    ret = sim.readCollision(collisionObjectHandle)
    _check_return(ret)
    return ret


def simReadDistance(distanceObjectHandle):
    result,    distanceData, objectHandlePair = \
        sim.checkDistance(distanceObjectHandle, sim.handle_all)
    return distanceData[7]


def simHandleDistance(distanceObjectHandle):
    return simReadDistance(distanceObjectHandle)


def simRemoveObject(objectHandle):
    ret = sim.removeObject(objectHandle)
    _check_return(ret)


def simRemoveModel(objectHandle):
    ret = sim.removeModel(objectHandle)
    _check_return(ret)


def simCloseScene():
    ret = sim.closeScene()
    _check_return(ret)


def simGetObjects(objectType):
    prev_handle = 0
    i = 0
    handles = []
    while prev_handle != -1:
        prev_handle = sim.getObjects(i, objectType)
        i += 1
        if prev_handle > -1:
            handles.append(prev_handle)
    return handles


def simSetObjectInt32Parameter(objectHandle, parameter, value):
    ret = sim.setObjectInt32Param(objectHandle, parameter, 1 if value else 0)
    _check_set_object_parameter(ret)
    _check_return(ret)


def simGetObjectInt32Parameter(objectHandle, parameter):
    return sim.getObjectInt32Param(objectHandle, parameter)


def simSetObjectFloatParameter(objectHandle, parameter, value):
    ret = sim.setObjectFloatParam(objectHandle, parameter, value)
    _check_set_object_parameter(ret)
    _check_return(ret)


def simGetObjectFloatParameter(objectHandle, parameter):
    return sim.getObjectFloatParam(objectHandle, parameter)

def simGetModelProperty(objectHandle):
    ret = sim.getModelProperty(objectHandle)
    _check_return(ret)
    return ret


def simSetModelProperty(objectHandle, prop):
    ret = sim.setModelProperty(objectHandle, prop)
    _check_return(ret)


def simGetObjectSpecialProperty(objectHandle):
    ret = sim.getObjectSpecialProperty(objectHandle)
    _check_return(ret)
    return ret


def simSetObjectSpecialProperty(objectHandle, prop):
    ret = sim.setObjectSpecialProperty(objectHandle, prop)
    _check_return(ret)


def simCreateDummy(size, color):
    ret = sim.createDummy(size, color)
    _check_return(ret)
    return ret


def simGetObjectVelocity(objectHandle):
    linearVel, angularVel = sim.getObjectVelocity(objectHandle)
    return linearVel, angularVel



def simExtCallScriptFunction(functionNameAtScriptName, scriptHandleOrType,
                             inputInts, inputFloats, inputStrings, inputBuffer):
    ret_ints,ret_floats,ret_strings,ret_buffer,_ = \
        lib.callScriptFunction(functionNameAtScriptName.encode('ascii'),
                               scriptHandleOrType, inputInts,
                               inputFloats, inputStrings)

    return ret_ints, ret_floats, ret_strings, ret_buffer


def simCreatePureShape(primitiveType, options, sizes):
    handle = sim.createPrimitiveShape(
        primitiveType,   sizes, options)
    _check_return(handle)
    return handle


def simGroupShapes(shapeHandles, merge=False):
    l = len(shapeHandles)
    handle = sim.groupShapes(shapeHandles, -l if merge else l)
    _check_return(handle)
    return handle


def simGetShapeColor(shapeHandle, colorName, colorComponent):

    return sim.getShapeColor(shapeHandle, colorName, colorComponent)



def simSetShapeColor(shapeHandle, colorName, colorComponent, rgbData):
    if colorName is None or len(colorName) == 0:
        colorName = None
    res = sim.setShapeColor(shapeHandle, colorName, colorComponent, rgbData)
    _check_return(res)


def simReorientShapeBoundingBox(shapeHandle, relativeToHandle):
    ret = sim.reorientShapeBoundingBox(shapeHandle, relativeToHandle, 0)
    _check_return(ret)


def simGetObjectMatrix(objectHandle, relativeToObjectHandle):
    return sim.getObjectMatrix(objectHandle, relativeToObjectHandle)



def simGetObjectsInTree(treeBaseHandle, objectType, options):
    handles = sim.getObjectsInTree(treeBaseHandle, objectType, options)
    return handles


def simGetExtensionString(objectHandle, index, key):
    ext = sim.getExtensionString(objectHandle, index, key.encode('ascii'))
    return ext


def simGetObjectType(objectHandle):
    ret = sim.getObjectType(objectHandle)
    _check_return(ret)
    return ret


def simGetConfigurationTree(objectHandle):
    config = sim.getConfigurationTree(objectHandle)
    _check_null_return(config)
    # TODO: Not use what to do about the encoding here
    # configs = ffi.string(config)
    # simReleaseBuffer(config)
    return config


def simSetConfigurationTree(data):
    ret = sim.setConfigurationTree(data)
    _check_return(ret)


def simRotateAroundAxis(matrix, axis, axisPos, angle):
    return sim.rotateAroundAxis(matrix, axis, axisPos, angle)

def simSetObjectMatrix(objectHandle, relativeToObjectHandle, matrix):
    ret = sim.setObjectMatrix(objectHandle, relativeToObjectHandle, matrix)
    _check_return(ret)


def simCheckCollision(entity1Handle, entity2Handle):
    state, collidingObjectHandles = sim.checkCollision(entity1Handle, entity2Handle)
    _check_return(state)
    return state


def simGetPositionOnPath(pathHandle, relativeDistance):
    return sim.getPositionOnPath(pathHandle, relativeDistance)



def simGetOrientationOnPath(pathHandle, relativeDistance):
    return sim.getOrientationOnPath(pathHandle, relativeDistance)

def simAddDrawingObject(objectType, size, duplicateTolerance,
                        parentObjectHandle, maxItemCount, ambient_diffuse=None,
                        specular=None, emission=None):
    """

    :param objectType: A drawing object type. e.g. sim_drawing_points.
    :param size: Size of the item. Width of lines or size of points are in
        pixels, other sizes are in meters.
    :param duplicateTolerance: If different from 0.0, then will only add the
        item if there is no other item within duplicateTolerance distance.
    :param parentObjectHandle: Handle of the scene object where the drawing
        items should keep attached to.
    :param maxItemCount: Maximum number of items this object can hold.
    :param ambient_diffuse: Ambient/diffuse color.
    :param specular: Default specular color.
    :param emission: Default emissive color.
    :return: Handle of the drawing object.
    """
    handle = sim.addDrawingObject(
        objectType, size, duplicateTolerance, parentObjectHandle, maxItemCount,
        ambient_diffuse, None, specular, emission)
    _check_return(handle)
    return handle


def simRemoveDrawingObject(objectHandle):
    ret = sim.removeDrawingObject(objectHandle)
    _check_return(ret)


def simAddDrawingObjectItem(objectHandle, itemData):
    """

    :param objectHandle: Handle of a drawing object.
    :param itemData: If the item is a point item, 3 values are required (x;y;z).
        If the item is a line item, 6 values are required, and if the item is a
        triangle item, 9 values are required. If None, then the drawing object
        is emptied of all its items.
    :return:
    """
    if itemData is None:
        itemData = None
    ret = sim.addDrawingObjectItem(objectHandle, itemData)
    _check_return(ret)


def simGetSimulationTimeStep():
    step = sim.getSimulationTimeStep()
    _check_return(step)
    return step


def simResetDynamicObject(objectHandle):
    ret = sim.resetDynamicObject(objectHandle)
    _check_return(ret)


def simGetJointType(objectHandle):
    type = sim.getJointType(objectHandle)
    _check_return(type)
    return type


def simRMLPos(dofs, smallestTimeStep, flags, currentPosVelAccel,
              maxVelAccelJerk, selection, targetPosVel):
    handle = sim.ruckigPos(
        dofs,
        0.0001,
        flags,
        currentPosVelAccel,
        maxVelAccelJerk,
        selection,
        targetPosVel)
    # handle = sim.simRMLPos(dofs, smallestTimeStep, flags, currentPosVelAccel,
    #                        maxVelAccelJerk, selection, targetPosVel)
    _check_return(handle)
    return handle


def simRMLVel(dofs, smallestTimeStep, flags, currentPosVelAccel, maxAccelJerk,
              selection, targetVel):

    handle = sim.ruckigVel(
        dofs,
        smallestTimeStep,
        flags,
        currentPosVelAccel,
        maxAccelJerk,
        selection,
        targetVel)
    # handle = lib.simRMLVel(dofs, smallestTimeStep, flags, currentPosVelAccel,
    #                        maxAccelJerk, selection, targetVel)
    _check_return(handle)
    return handle


def simRMLStep(handle, timeStep, dofs):
    # timeStep = ffi.cast('double', timeStep)
    result, newPosVelAccel, synchronizationTime = sim.ruckigStep(handle, timeStep)
    # state , newPosVelAccel,_ = sim.simRMLStep(handle, timeStep)
    # _check_return(state)
    return result, newPosVelAccel


def simRMLRemove(handle):
    sim.ruckigRemove(handle)


def simImportMesh(fileformat, pathAndFilename, options,
                  identicalVerticeTolerance, scalingFactor):

    outVerticies,outIndices,outNames  = sim.importMesh(
        fileformat, pathAndFilename.encode('ascii'), options,
        identicalVerticeTolerance, scalingFactor)
    return outVerticies,outIndices,outNames


def simImportShape(fileformat, pathAndFilename, options,
                   identicalVerticeTolerance, scalingFactor):
    handle = sim.importShape(
        fileformat, pathAndFilename.encode('ascii'), options,
        identicalVerticeTolerance, scalingFactor)
    _check_return(handle)
    return handle


def simCreateMeshShape(options, shadingAngle, vertices, indices):
    ret = sim.createMeshShape(options, shadingAngle, vertices, len(vertices),
                                 indices, len(indices), None)
    return ret


def simGetShapeMesh(shapeHandle):
    outVerticies,    outIndices,   outNormals = sim.getShapeMesh(shapeHandle)
    return outVerticies,    outIndices,   outNormals


def simGetShapeViz(shapeHandle, index):

    info = sim.getShapeViz(shapeHandle, index)

    return info


def simConvexDecompose(shapeHandle, options, intParams, floatParams):
    return sim.convexDecompose(shapeHandle, options, intParams, floatParams)


def simGetJointMode(shapeHandle):
    mode = sim.getJointMode(shapeHandle)
    _check_return(mode)
    return mode


def simSetJointMode(shapeHandle, mode):
    options = 0
    ret = sim.setJointMode(shapeHandle, mode, options)
    _check_return(ret)


def simCreatePath(attributes, intParams, floatParams, color):
    handle = sim.createPath(attributes, intParams, floatParams, color + [0.]*3 + [0.25]*3 + [0.]*3)
    _check_return(handle)
    return handle


def simAddScript(type):
    handle = sim.addScript(type)
    _check_return(handle)
    return handle


def simAssociateScriptWithObject(scriptHandle, objectHandle):
    ret = sim.associateScriptWithObject(scriptHandle, objectHandle)
    _check_return(ret)


def simSetScriptText(scriptHandle, scriptText):
    ret = sim.setScriptText(scriptHandle, scriptText.encode('ascii'))
    _check_return(ret)


def simGetScriptText(scriptHandle):
    ret = sim.getScriptText(scriptHandle)
    return ret


def simGetScriptAssociatedWithObject(objectHandle):
    ret = sim.getScriptAssociatedWithObject(objectHandle)
    return ret


def simApplyTexture(shapeHandle, textureCoordinates, textCoordSize,
                    texture, textureResolution, options):
    ret = sim.applyTexture(shapeHandle, textureCoordinates, textCoordSize,
                              texture, textureResolution, options)
    _check_return(ret)
    return ret


def simCreateTexture(fileName, options):
    # The textureID param that is returned from simCreateTexture seems
    # to be incorrect (in regards to calling simGetShapeTextureId on the
    # generated plane).
    handle = sim.createTexture(fileName.encode('ascii'), options, None,
                                  None, None, 0, None, None,
                                  None)
    _check_return(handle)
    return handle


def simSetShapeTexture(shapeHandle, textureId, mappingMode, options, uvScaling,
                       position, orientation):


    handle = sim.setShapeTexture(shapeHandle, textureId, mappingMode,
                                    options, uvScaling, position, orientation)
    _check_return(handle)


def simGetShapeTextureId(objectHandle):
    ret = sim.getShapeTextureId(objectHandle)
    _check_return(ret)
    return ret


def simCopyPasteObjects(objectHandles, options):
    handles = sim.copyPasteObjects( objectHandles )
    return handles


def simHandleIkGroup(environmentHandle, ikGroupHandle):

    ret,  flags, precision = simIK.handleGroup(
        environmentHandle,
        ikGroupHandle )
    # ret = lib.simHandleIkGroup(ikGroupHandle)
    _check_return(ret)
    return ret


def simCheckIkGroup(environmentHandle, ikGroupHandle, jointHandles):
    jointPositions = simIK.findConfig(
        environmentHandle,
        ikGroupHandle,
        jointHandles)
    return 1, jointPositions


def simComputeJacobian(environmentHandle, ikGroupHandle, options):
    # Only works when joints that are in IK or hybrid mode
    jacobian, errorVector = simIK.computeGroupJacobian(environmentHandle,
                                 ikGroupHandle)
    # _check_return(ret)
    return jacobian, errorVector



def simGetIkGroupMatrix(ikGroupHandle, options):
    ret,matrixSize = sim.getIkGroupMatrix(ikGroupHandle, options)
    flatJacobian = [ret[i] for i in range(matrixSize[0] * matrixSize[1])]
    # matrixSize[0] represents the row count of the Jacobian.
    # matrixSize[0] represents the column count of the Jacobian.
    return flatJacobian, matrixSize


def simCheckDistance(entity1Handle, entity2Handle, threshold):
    return sim.checkDistance(
        entity1Handle, entity2Handle, threshold)



def simSetExplicitHandling(generalObjectHandle, explicitFlags):
    ret = sim.setExplicitHandling(generalObjectHandle, explicitFlags)
    _check_return(ret)


def simGetExplicitHandling(generalObjectHandle):
    flag = sim.getExplicitHandling(generalObjectHandle)
    _check_return(flag)
    return flag


def simUngroupShape(shapeHandle):
    return  sim.ungroupShape(shapeHandle)



def simInvertMatrix(matrix):
    c_matrix = lib.getMatrixInverse(matrix)
    return c_matrix


def simMultiplyMatrices(inMatrix1, inMatrix2):
    return sim.multiplyMatrices(inMatrix1, inMatrix2)


def simGetEulerAnglesFromMatrix(rotationMatrix):
    return sim.getEulerAnglesFromMatrix(rotationMatrix)



def simGetSimulationTime():
    time = sim.getSimulationTime()
    _check_return(time)
    return time


def simSetIntegerSignal(signalName, signalValue):
    ret = sim.setIntegerSignal(signalName.encode('ascii'), signalValue)
    _check_return(ret)


def simGetIntegerSignal(signalName):
    val = sim.getInt32Signal(signalName.encode('ascii'))
    return 1, val


def simClearIntegerSignal(signalName):
    ret = sim.clearInt32Signal(signalName.encode('ascii'))
    _check_return(ret)
    return ret


def simSetFloatSignal(signalName, signalValue):
    ret = sim.setFloatSignal(signalName.encode('ascii'), signalValue)
    _check_return(ret)


def simGetFloatSignal(signalName):
    val = sim.getFloatSignal(signalName.encode('ascii'))
    return 1, val


def simClearFloatSignal(signalName):
    ret = sim.clearFloatSignal(signalName.encode('ascii'))
    _check_return(ret)
    return ret


def simSetDoubleSignal(signalName, signalValue):
    ret = sim.setDoubleSignal(signalName.encode('ascii'), signalValue)
    _check_return(ret)


def simGetDoubleSignal(signalName):
    val = sim.getDoubleSignal(signalName.encode('ascii'))
    return 1, val


def simClearDoubleSignal(signalName):
    ret = sim.clearDoubleSignal(signalName.encode('ascii'))
    _check_return(ret)
    return ret


def simSetStringSignal(signalName, signalValue):
    ret = sim.setStringSignal(
        signalName.encode('ascii'), signalValue.encode('ascii'),
        len(signalValue))
    _check_return(ret)


def simGetStringSignal(signalName):
    str_ret = sim.getStringSignal(signalName.encode('ascii'))
    if str_ret == None:
        # No value.
        return 0, None
    return 1, str_ret


def simClearStringSignal(signalName):
    ret = sim.clearStringSignal(signalName.encode('ascii'))
    _check_return(ret)
    return ret


def simSetUserParameter(objectHandle, parameterName, parameterValue):
    # TODO: currently not used by PyRep.
    # User params functionality missing in CoppeliaSim.
    parameterLength = len(parameterValue)
    ret = sim.setUserParam(
        objectHandle, parameterName.encode('ascii'),
        parameterValue.encode('ascii'), parameterLength)
    _check_return(ret)


def simGetUserParameter(objectHandle, parameterName):
    # TODO: currently not used by PyRep.
    # User params functionality missing in CoppeliaSim.
    parameterValue = sim.getUserParam(
        objectHandle, parameterName.encode('ascii'))
    _check_null_return(parameterValue)
    return parameterValue


def simCreateOctree(voxelSize, options, pointSize):
    ret = sim.createOctree(voxelSize, options, pointSize, None)
    _check_return(ret)
    return ret


def simInsertVoxelsIntoOctree(octreeHandle, options, points, color, tag):
    if color is None:
        color = None
    if tag is None:
        tag = None
    ret = sim.insertVoxelsIntoOctree(octreeHandle, options, points,
                                        len(points)//3, color, tag, None)
    _check_return(ret)
    return ret


def simRemoveVoxelsFromOctree(octreeHandle, options, points):
    if points is None:
        points = None
    if points is None:
        pointCount = 0
    else:
        pointCount = len(points)//3
    ret = sim.removeVoxelsFromOctree(octreeHandle, options, points,
                                        pointCount, None)
    _check_return(ret)
    return ret


def simGetOctreeVoxels(octreeHandle):
    return  sim.getOctreeVoxels(octreeHandle)



def simInsertObjectIntoOctree(octreeHandle, objectHandle, options,
                              color, tag):
    if color is None:
        color = None
    ret = sim.insertObjectIntoOctree(octreeHandle, objectHandle, options,
                                        color, tag, None)
    _check_return(ret)
    return ret


def simSubtractObjectFromOctree(octreeHandle, objectHandle, options):
    ret = sim.subtractObjectFromOctree(octreeHandle, objectHandle, options,
                                        None)
    _check_return(ret)
    return ret


def simCheckOctreePointOccupancy(octreeHandle, options, points):
    ret = sim.checkOctreePointOccupancy(octreeHandle, options, points,
                                           len(points)//3, None, None,
                                           None)
    _check_return(ret)
    if ret == 1:
        return True
    else:
        return False


def simGetContactInfo(contact_obj_handle, get_contact_normal):
    index = 0
    contact_list = []
    result = 1

    while result > 0:
        if get_contact_normal:
            ext = sim_handleflag_extended
        else:
            ext = 0

        collidingObjects,collisionPoint,reactionForce,normalVector =\
            sim.getContactInfo(sim_handle_all, contact_obj_handle, index + ext)

        contact_info = {
            "contact": collisionPoint,
            "contact_handles": collidingObjects,
            "contact_normal": normalVector
        }
        contact_list.append(contact_info)
        index += 1
    contact_list.pop(-1)  # remove the all zero value
    return contact_list


def simGetConfigForTipPose(ikEnv, ikGroupHandle, jointHandles, thresholdDist, maxTimeInMs, metric, collisionPairs, jointOptions, lowLimits, ranges):

    # jointPositions = sim.getConfigForTipPose(
    #     ikGroupHandle,
    #     jointHandles,
    #     thresholdDist,
    #     maxTimeInMs,
    #     [1,1,1,0.1],
    #     collisionPairs ,
    #     jointOptions ,
    #     lowLimits  ,
    #     ranges  )
    jointPositions = simIK.findConfig(ikEnv,  ikGroupHandle, jointHandles, thresholdDist, maxTimeInMs / 1000, [1, 1, 1, 0.1])
    # ignoring collisions
    # metric = [1, 1, 1, 0.1]
    return jointPositions


def generateIkPath(ikEnv, ikGroupHandle, jointHandles, ptCnt, collisionPairs, jointOptions, tip):
    collisionPairs = None if len(collisionPairs) == 0 else collisionPairs
    joints = simIK.generatePath(ikEnv,
        ikGroupHandle, jointHandles, tip, ptCnt)
    return joints


def simGetDecimatedMesh(inVertices, inIndices, decimationPercent):

    # outNormals is 3 times the size of outIndicesCount
    # outNormals = ffi.new('float **')

    ret, outVerticies, \
        outIndices, = \
        sim.getDecimatedMesh(inVertices, len(inVertices),
                                  inIndices, len(inIndices),
                                  decimationPercent, 0, None)
    _check_return(ret)

    return outVerticies, outIndices


def simComputeMassAndInertia(shapeHandle, density):
    ret = sim.computeMassAndInertia(shapeHandle, density)
    _check_return(ret)
    return ret


def simAddForce(shapeHandle, position, force):
    ret = sim.addForce(shapeHandle, position, force)
    _check_return(ret)


def simAddForceAndTorque(shapeHandle, force, torque):
    ret = sim.addForceAndTorque(shapeHandle,
                                   None if force is None else force,
                                   None if torque is None else torque)
    _check_return(ret)


def simSetLightParameters(shapeHandle, state, diffusePart=None, specularPart=None):
    ret = sim.setLightParameters(shapeHandle, state, None,
                                    None if diffusePart is None else diffusePart,
                                    None if specularPart is None else specularPart)
    _check_return(ret)


def simGetLightParameters(shapeHandle):

    ret,diffusePart,specularPart = sim.getLightParameters(shapeHandle, None)
    _check_return(ret)
    return ret, list(diffusePart), list(specularPart)
