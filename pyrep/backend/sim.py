from .simConst import *
from ._sim_cffi import ffi, lib
import numpy as np
import collections


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
    if val == ffi.NULL:
        raise RuntimeError(
            'The call failed on the V-REP side by returning null.')


def _check_set_object_parameter(ret):
    if ret == 0:
        raise RuntimeError(
            'Parameter could not be set (e.g. because the parameterID doesn\'t '
            'exist, or because the specified object doesn\'t correspond to the '
            'correct type)')


def simExtLaunchUIThread(options, scene, pyrep_root):
    lib.simExtLaunchUIThread(
        'PyRep'.encode('ascii'), options, scene.encode('ascii'),
        pyrep_root.encode('ascii'))


def simExtSimThreadInit():
    lib.simExtSimThreadInit()


def simExtCanInitSimThread():
    return bool(lib.simExtCanInitSimThread())


def simExtSimThreadDestroy():
    lib.simExtSimThreadDestroy()


def simExtPostExitRequest():
    lib.simExtPostExitRequest()


def simExtGetExitRequest():
    return bool(lib.simExtGetExitRequest())


def simExtStep(stepIfRunning=True):
    lib.simExtStep(stepIfRunning)


def simStartSimulation():
    lib.simStartSimulation()


def simStopSimulation():
    lib.simStopSimulation()


def simPauseSimulation():
    return lib.simPauseSimulation()


def simQuitSimulator(doNotDisplayMessages):
    lib.simQuitSimulator(doNotDisplayMessages)


def simGetObjectHandle(objectName):
    handle = lib.simGetObjectHandle(objectName.encode('ascii'))
    if handle < 0:
        raise RuntimeError('Handle %s does not exist.' % objectName)
    return handle


def simGetIkGroupHandle(ikGroupName):
    handle = lib.simGetIkGroupHandle(ikGroupName.encode('ascii'))
    if handle <= 0:
        raise RuntimeError('Ik group does not exist.')
    return handle


def simSetIkElementProperties(ikGroupHandle, tipDummyHandle, constraints,
                              precision=None, weight=None):
    if precision is None:
        precision = ffi.NULL
    if weight is None:
        weight = ffi.NULL
    reserved = ffi.NULL
    ret = lib.simSetIkElementProperties(
        ikGroupHandle, tipDummyHandle, constraints, precision, weight, reserved)
    _check_return(ret)
    return ret


def simSetIkGroupProperties(ikGroupHandle, resolutionMethod, maxIterations, damping):
    reserved = ffi.NULL
    ret = lib.simSetIkGroupProperties(
        ikGroupHandle, resolutionMethod, maxIterations, damping, reserved)
    _check_return(ret)
    return ret


def simGetObjectPosition(objectHandle, relativeToObjectHandle):
    position = ffi.new('float[3]')
    lib.simGetObjectPosition(objectHandle, relativeToObjectHandle, position)
    return list(position)


def simGetJointPosition(jointHandle):
    position = ffi.new('float *')
    lib.simGetJointPosition(jointHandle, position)
    return position[0]


def simSetJointPosition(jointHandle, position):
    lib.simSetJointPosition(jointHandle, position)


def simGetJointMatrix(jointHandle):
    matrix = ffi.new('float[12]')
    lib.simGetJointMatrix(jointHandle, matrix)
    return list(matrix)


def simSetSphericalJointMatrix(jointHandle, matrix):
    lib.simSetSphericalJointMatrix(jointHandle, matrix)


def simGetJointTargetVelocity(jointHandle):
    vel = ffi.new('float *')
    lib.simGetJointTargetVelocity(jointHandle, vel)
    return vel[0]


def simSetJointTargetVelocity(jointHandle, targetVelocity):
    lib.simSetJointTargetVelocity(jointHandle, targetVelocity)


def simGetJointTargetPosition(jointHandle):
    position = ffi.new('float *')
    lib.simGetJointTargetPosition(jointHandle, position)
    return position[0]


def simSetJointTargetPosition(jointHandle, targetPosition):
    lib.simSetJointTargetPosition(jointHandle, targetPosition)


def simGetJointForce(jointHandle):
    force = ffi.new('float *')
    ret = lib.simGetJointForce(jointHandle, force)
    _check_return(ret)
    if ret == 0:
        raise RuntimeError('No value available yet.')
    return force[0]


def simSetJointForce(jointHandle, force):
    lib.simSetJointForce(jointHandle, force)


def simGetJointMaxForce(jointHandle):
    force = ffi.new('float *')
    ret = lib.simGetJointMaxForce(jointHandle, force)
    _check_return(ret)
    if ret == 0:
        raise RuntimeError('No value available yet.')
    return force[0]


def simSetJointMaxForce(jointHandle, force):
    lib.simSetJointMaxForce(jointHandle, force)


def simGetJointInterval(jointHandle):
    cyclic = ffi.new('char *')
    interval = ffi.new('float [2]')
    ret = lib.simGetJointInterval(jointHandle, cyclic, interval)
    _check_return(ret)
    return ffi.string(cyclic).decode('utf-8') != '', list(interval)


def simSetJointInterval(jointHandle, cyclic, interval):
    ret = lib.simSetJointInterval(jointHandle, cyclic, interval)
    _check_return(ret)


def simCreateForceSensor(options, intParams, floatParams, color):
    if color is None:
        color = ffi.NULL
    handle = lib.simCreateForceSensor(options, intParams, floatParams, color)
    _check_return(handle)
    return handle


def simBreakForceSensor(forceSensorHandle):
    lib.simBreakForceSensor(forceSensorHandle)


def simReadForceSensor(forceSensorHandle):
    forceVector  = ffi.new('float[3]')
    torqueVector = ffi.new('float[3]')
    state = lib.simReadForceSensor(forceSensorHandle, forceVector, torqueVector)
    return state, list(forceVector), list(torqueVector)


def simReleaseBuffer(pointer):
    lib.simReleaseBuffer(pointer)


def simCreateVisionSensor(options, intParams, floatParams, color):
    if color is None:
        color = ffi.NULL
    ret = lib.simCreateVisionSensor(options, intParams, floatParams, color)
    _check_return(ret)
    return ret


def simHandleVisionSensor(sensorHandle):
    auxValues = ffi.new('float **')
    auxValuesCount = ffi.new('int **')
    ret = lib.simHandleVisionSensor(
        sensorHandle, auxValues, auxValuesCount)
    _check_return(ret)

    k1 = 0
    outAuxValues = []
    for i in range(auxValuesCount[0][0]):
        k2 = k1 + auxValuesCount[0][i + 1]
        outAuxValues.extend([x for x in auxValues[0][k1:k2]])

    simReleaseBuffer(ffi.cast('char *', auxValues[0]))
    simReleaseBuffer(ffi.cast('char *', auxValuesCount[0]))
    return ret, outAuxValues


def simReadVisionSensor(sensorHandle):
    auxValues = ffi.new('float **')
    auxValuesCount = ffi.new('int **')
    state = lib.simReadVisionSensor(sensorHandle, auxValues, auxValuesCount)
    auxValues2 = []
    if state == 0:
        s = 0
        for i in range(auxValuesCount[0]):
            auxValues2.append(auxValues[s:s+auxValuesCount[i+1]])
            s += auxValuesCount[i+1]
        #free C buffers
        simReleaseBuffer(auxValues)
        simReleaseBuffer(auxValuesCount)
    return state, auxValues2


def simGetVisionSensorImage(sensorHandle, resolution):
    img_buffer = lib.simGetVisionSensorImage(sensorHandle)
    T = ffi.getctype(ffi.typeof(img_buffer).item)  # Buffer data type
    s = ffi.sizeof(T)  # datatype size
    np_T = np.dtype('f{:d}'.format(s))  # Numpy equivalent, e.g. float is 'f4'
    # Wrap the buffer with numpy.
    img = np.frombuffer(ffi.buffer(img_buffer, resolution[0]*resolution[1]*3*s), np_T)
    img = img.reshape(resolution[1], resolution[0], 3)
    img = np.flip(img, 0).copy()  # Image is upside-down
    simReleaseBuffer(ffi.cast('char *', img_buffer))
    return img


def simGetVisionSensorDepthBuffer(sensorHandle, resolution, in_meters):
    if in_meters:
        sensorHandle += sim_handleflag_depthbuffermeters
    img_buffer = lib.simGetVisionSensorDepthBuffer(sensorHandle)
    T = ffi.getctype(ffi.typeof(img_buffer).item)  # Buffer data type
    s = ffi.sizeof(T)  # datatype size
    np_T = np.dtype('f{:d}'.format(s))  # Numpy equivalent, e.g. float is 'f4'
    # Wrap the buffer with numpy.
    img = np.frombuffer(ffi.buffer(img_buffer, resolution[0]*resolution[1]*s), np_T)
    img = img.reshape(resolution[1], resolution[0])
    img = np.flip(img, 0).copy()  # Image is upside-down
    simReleaseBuffer(ffi.cast('char *', img_buffer))
    return img


def simGetVisionSensorResolution(sensorHandle):
    resolution = ffi.new('int[2]')
    ret = lib.simGetVisionSensorResolution(sensorHandle, resolution)
    _check_return(ret)
    return list(resolution)


def simGetObjectChild(parentObjectHandle, childIndex):
    val = lib.simGetObjectChild(parentObjectHandle, childIndex)
    _check_return(val)
    return val


def simGetObjectParent(childObjectHandle):
    val = lib.simGetObjectParent(childObjectHandle)
    _check_return(val)
    return val


def simReadProximitySensor(sensorHandle):
    detectedPoint = ffi.new('float[3]')
    detectedObjectHandle = ffi.new('int *')
    detectedSurfaceNormalVector = ffi.new('float[3]')
    state = lib.simReadProximitySensor(
        sensorHandle, detectedPoint, detectedObjectHandle,
        detectedSurfaceNormalVector)
    _check_return(state)
    return (state, detectedObjectHandle, list(detectedPoint),
            list(detectedSurfaceNormalVector))


def simCheckProximitySensor(sensorHandle, entityHandle):
    detectedPoint = ffi.new('float[3]')
    state = lib.simCheckProximitySensor(
        sensorHandle, entityHandle, detectedPoint)
    _check_return(state)
    return state, list(detectedPoint)


def simLoadModel(modelPathAndName):
    val = lib.simLoadModel(modelPathAndName.encode('ascii'))
    _check_return(val)
    return val


def simLoadScene(scenePathAndName):
    val = lib.simLoadScene(scenePathAndName.encode('ascii'))
    _check_return(val)
    return val


def simSaveModel(modelHandle, modelPathAndName):
    val = lib.simSaveModel(modelHandle, modelPathAndName.encode('ascii'))
    _check_return(val)
    return val


def simSaveScene(filename):
    val = lib.simSaveScene(filename.encode('ascii'))
    _check_return(val)
    return val


def simGetObjectName(objectHandle):
    name_raw = lib.simGetObjectName(objectHandle)
    if name_raw == ffi.NULL:
        return ''
    name = ffi.string(name_raw).decode('utf-8')
    simReleaseBuffer(name_raw)
    return name


def simSetObjectName(objectHandle, name):
    ret = lib.simSetObjectName(objectHandle, name.encode('ascii'))
    _check_return(ret)


def simAddStatusbarMessage(message):
    return lib.simAddStatusbarMessage(message.encode('ascii'))


def simGetObjectOrientation(objectHandle, relativeToObjectHandle):
    eulerAngles = ffi.new('float[3]')
    ret = lib.simGetObjectOrientation(
        objectHandle, relativeToObjectHandle, eulerAngles)
    _check_return(ret)
    return list(eulerAngles)


def simGetObjectQuaternion(objectHandle, relativeToObjectHandle):
    quaternion = ffi.new('float[4]')
    ret = lib.simGetObjectQuaternion(
        objectHandle, relativeToObjectHandle, quaternion)
    _check_return(ret)
    return list(quaternion)


def simSetObjectOrientation(objectHandle, relativeToObjectHandle, eulerAngles):
    ret = lib.simSetObjectOrientation(
        objectHandle, relativeToObjectHandle, eulerAngles)
    _check_return(ret)


def simSetObjectQuaternion(objectHandle, relativeToObjectHandle, quaternion):
    ret = lib.simSetObjectQuaternion(
        objectHandle, relativeToObjectHandle, quaternion)
    _check_return(ret)


def simSetObjectPosition(objectHandle, relativeToObjectHandle, position):
    ret = lib.simSetObjectPosition(
        objectHandle, relativeToObjectHandle, position)
    _check_return(ret)


def simSetObjectParent(objectHandle, parentObject, keepInPlace):
    ret = lib.simSetObjectParent(objectHandle, parentObject, keepInPlace)
    _check_return(ret)


def simGetArrayParameter(paramIdentifier):
    paramValues = ffi.new('float[3]')
    ret = lib.simGetArrayParameter(paramIdentifier, paramValues)
    _check_return(ret)
    return list(paramValues)


def simSetArrayParameter(paramIdentifier, paramValues):
    ret = lib.simSetArrayParameter(paramIdentifier, paramValues)
    _check_return(ret)


def simGetBoolParameter(parameter):
    ret = lib.simGetBoolParameter(parameter)
    _check_return(ret)
    return ret


def simSetBoolParameter(parameter, value):
    ret = lib.simSetBoolParameter(parameter, value)
    _check_return(ret)


def simGetInt32Parameter(parameter):
    ret = lib.simGetInt32Parameter(parameter)
    _check_return(ret)
    return ret


def simSetInt32Parameter(parameter, value):
    ret = lib.simSetInt32Parameter(parameter, value)
    _check_return(ret)


def simGetFloatParameter(parameter):
    ret = lib.simGetFloatParameter(parameter)
    _check_return(ret)
    return ret


def simSetFloatParameter(parameter, value):
    ret = lib.simSetFloatParameter(parameter, value)
    _check_return(ret)


def simSetStringParameter(parameter, value):
    ret = lib.simSetStringParameter(parameter, value.encode('ascii'))
    _check_return(ret)


def simGetStringParameter(parameter):
    val = lib.simGetStringParameter(parameter)
    _check_null_return(val)
    sval = ffi.string(val).decode('utf-8')
    simReleaseBuffer(val)
    return sval


def simGetEngineFloatParameter(parameter, objectHandle):
    ok = ffi.new('unsigned char *')
    ret = lib.simGetEngineFloatParameter(parameter, objectHandle, ffi.NULL, ok)
    _check_return(ret)
    return ret


def simSetEngineFloatParameter(parameter, objectHandle, val):
    ret = lib.simSetEngineFloatParameter(parameter, objectHandle, ffi.NULL,
                                         val)
    _check_return(ret)
    return ret


def simGetCollisionHandle(collisionObjectName):
    ret = lib.simGetCollisionHandle(collisionObjectName.encode('ascii'))
    _check_return(ret)
    return ret


def simGetCollectionHandle(collectionName):
    ret = lib.simGetCollectionHandle(collectionName.encode('ascii'))
    _check_return(ret)
    return ret


def simGetDistanceHandle(distanceObjectName):
    ret = lib.simGetDistanceHandle(distanceObjectName.encode('ascii'))
    _check_return(ret)
    return ret


def simReadCollision(collisionObjectHandle):
    ret = lib.simReadCollision(collisionObjectHandle)
    _check_return(ret)
    return ret


def simReadDistance(distanceObjectHandle):
    minimumDistance = ffi.new('float *')
    ret = lib.simReadDistance(distanceObjectHandle, minimumDistance)
    _check_return(ret)
    return minimumDistance[0]


def simHandleDistance(distanceObjectHandle):
    minimumDistance = ffi.new('float *')
    ret = lib.simHandleDistance(distanceObjectHandle, minimumDistance)
    _check_return(ret)
    return ret


def simRemoveObject(objectHandle):
    ret = lib.simRemoveObject(objectHandle)
    _check_return(ret)


def simRemoveModel(objectHandle):
    ret = lib.simRemoveModel(objectHandle)
    _check_return(ret)


def simCloseScene():
    ret = lib.simCloseScene()
    _check_return(ret)


def simGetObjects(objectType):
    prev_handle = 0
    i = 0
    handles = []
    while prev_handle != -1:
        prev_handle = lib.simGetObjects(i, objectType)
        i += 1
        if prev_handle > -1:
            handles.append(prev_handle)
    return handles


def simSetObjectInt32Parameter(objectHandle, parameter, value):
    ret = lib.simSetObjectInt32Parameter(objectHandle, parameter, value)
    _check_set_object_parameter(ret)
    _check_return(ret)


def simGetObjectInt32Parameter(objectHandle, parameter):
    value = ffi.new('int *')
    ret = lib.simGetObjectInt32Parameter(objectHandle, parameter, value)
    _check_set_object_parameter(ret)
    _check_return(ret)
    return value[0]


def simSetObjectFloatParameter(objectHandle, parameter, value):
    ret = lib.simSetObjectFloatParameter(objectHandle, parameter, value)
    _check_set_object_parameter(ret)
    _check_return(ret)


def simGetObjectFloatParameter(objectHandle, parameter):
    value = ffi.new('float *')
    ret = lib.simGetObjectFloatParameter(objectHandle, parameter, value)
    _check_set_object_parameter(ret)
    _check_return(ret)
    return value[0]


def simGetModelProperty(objectHandle):
    ret = lib.simGetModelProperty(objectHandle)
    _check_return(ret)
    return ret


def simSetModelProperty(objectHandle, prop):
    ret = lib.simSetModelProperty(objectHandle, prop)
    _check_return(ret)


def simGetObjectSpecialProperty(objectHandle):
    ret = lib.simGetObjectSpecialProperty(objectHandle)
    _check_return(ret)
    return ret


def simSetObjectSpecialProperty(objectHandle, prop):
    ret = lib.simSetObjectSpecialProperty(objectHandle, prop)
    _check_return(ret)


def simCreateDummy(size, color):
    if color is None:
        color = ffi.NULL
    ret = lib.simCreateDummy(size, color)
    _check_return(ret)
    return ret


def simGetObjectVelocity(objectHandle):
    linearVel = ffi.new('float[3]')
    angularVel = ffi.new('float[3]')
    ret = lib.simGetObjectVelocity(objectHandle, linearVel, angularVel)
    _check_return(ret)
    return list(linearVel), list(angularVel)


def simCreateStack():
    ret = lib.simCreateStack()
    _check_return(ret)
    return ret


def simReleaseStack(stackHandle):
    ret = lib.simReleaseStack(stackHandle)
    _check_return(ret)


def simPushInt32OntoStack(stackHandle, value):
    ret = lib.simPushInt32OntoStack(stackHandle, value)
    _check_return(ret)


def simGetStackInt32Value(stackHandle):
    value = ffi.new('int *')
    ret = lib.simGetStackInt32Value(stackHandle, value)
    _check_return(ret)
    return value[0]


def simPushFloatOntoStack(stackHandle, value):
    ret = lib.simPushFloatOntoStack(stackHandle, value)
    _check_return(ret)


def simGetStackFloatValue(stackHandle):
    value = ffi.new('float *')
    ret = lib.simGetStackFloatValue(stackHandle, value)
    _check_return(ret)
    return value[0]


def simPushStringOntoStack(stackHandle, value):
    ret = lib.simPushStringOntoStack(stackHandle, value.encode('ascii'), 0)
    _check_return(ret)


def simGetStackStringValue(stackHandle):
    val = lib.simGetStackFloatValue(stackHandle, ffi.NULL)
    _check_null_return(val)
    sval = ffi.string(val).decode('utf-8')
    simReleaseBuffer(val)
    return sval


def simExtCallScriptFunction(functionNameAtScriptName, scriptHandleOrType,
                             inputInts, inputFloats, inputStrings, inputBuffer):
    char_pointers = []
    for s in inputStrings:
        char_pointers.append(ffi.new('char[]', s.encode('ascii')))
    strIn = ffi.new('char *[]', char_pointers)
    outInt = ffi.new('int **')
    outIntCnt = ffi.new('int *')
    outFloat = ffi.new('float **')
    outFloatCnt = ffi.new('int *')
    outString = ffi.new('char ***')
    outStringCnt = ffi.new('int *')
    outBuffer = ffi.new('char **')
    outBufferSize = ffi.new('int *')

    ret = lib.simExtCallScriptFunction(
        scriptHandleOrType, functionNameAtScriptName.encode('ascii'),
        inputInts, len(inputInts),
        inputFloats, len(inputFloats),
        strIn, len(inputStrings),
        str(inputBuffer).encode('ascii'), len(str(inputBuffer)),
        outInt, outIntCnt, outFloat, outFloatCnt,
        outString, outStringCnt, outBuffer, outBufferSize)
    _check_return(ret)

    ret_ints = [outInt[0][i] for i in range(outIntCnt[0])]
    ret_floats = [outFloat[0][i] for i in range(outFloatCnt[0])]
    ret_strings = [ffi.string(outString[0][i]).decode('utf-8')
                   for i in range(outStringCnt[0])]
    ret_buffer = ''
    if outBufferSize[0] > 0:
        ret_buffer = ffi.string(outBuffer[0]).decode('utf-8')

    simReleaseBuffer(ffi.cast('char *', outInt[0]))
    simReleaseBuffer(ffi.cast('char *', outFloat[0]))
    [simReleaseBuffer(outString[0][i]) for i in range(outStringCnt[0])]
    simReleaseBuffer(outBuffer[0])

    return ret_ints, ret_floats, ret_strings, ret_buffer


def simCreatePureShape(primitiveType, options, sizes, mass, precision):
    if precision is None:
        precision = ffi.NULL
    handle = lib.simCreatePureShape(
        primitiveType, options, sizes, mass, precision)
    _check_return(handle)
    return handle


def simGroupShapes(shapeHandles, merge=False):
    l = len(shapeHandles)
    handle = lib.simGroupShapes(shapeHandles, -l if merge else l)
    _check_return(handle)
    return handle


def simGetShapeColor(shapeHandle, colorName, colorComponent):
    rgbData = ffi.new('float[3]')
    if colorName is None or len(colorName) == 0:
        colorName = ffi.NULL
    res = lib.simGetShapeColor(shapeHandle, colorName, colorComponent, rgbData)
    _check_return(res)
    return list(rgbData)


def simSetShapeColor(shapeHandle, colorName, colorComponent, rgbData):
    if colorName is None or len(colorName) == 0:
        colorName = ffi.NULL
    res = lib.simSetShapeColor(shapeHandle, colorName, colorComponent, rgbData)
    _check_return(res)


def simReorientShapeBoundingBox(shapeHandle, relativeToHandle):
    ret = lib.simReorientShapeBoundingBox(shapeHandle, relativeToHandle, 0)
    _check_return(ret)


def simGetObjectMatrix(objectHandle, relativeToObjectHandle):
    matrix = ffi.new('float[12]')
    ret = lib.simGetObjectMatrix(objectHandle, relativeToObjectHandle, matrix)
    _check_return(ret)
    return list(matrix)


def simGetObjectsInTree(treeBaseHandle, objectType, options):
    objectCount = ffi.new('int *')
    handles = lib.simGetObjectsInTree(treeBaseHandle, objectType, options,
                                      objectCount)
    _check_null_return(handles)
    ret = [handles[i] for i in range(objectCount[0])]
    simReleaseBuffer(ffi.cast('char *', handles))
    return ret


def simGetExtensionString(objectHandle, index, key):
    ext = lib.simGetExtensionString(objectHandle, index, key.encode('ascii'))
    if ext == ffi.NULL:
        return ''
    exts = ffi.string(ext).decode('utf-8')
    simReleaseBuffer(ext)
    return exts


def simGetObjectType(objectHandle):
    ret = lib.simGetObjectType(objectHandle)
    _check_return(ret)
    return ret


def simGetConfigurationTree(objectHandle):
    config = lib.simGetConfigurationTree(objectHandle)
    _check_null_return(config)
    # TODO: Not use what to do about the encoding here
    # configs = ffi.string(config)
    # simReleaseBuffer(config)
    return config


def simSetConfigurationTree(data):
    ret = lib.simSetConfigurationTree(data)
    _check_return(ret)


def simRotateAroundAxis(matrix, axis, axisPos, angle):
    matrixOut = ffi.new('float[12]')
    ret = lib.simRotateAroundAxis(matrix, axis, axisPos, angle, matrixOut)
    _check_return(ret)
    return list(matrixOut)


def simSetObjectMatrix(objectHandle, relativeToObjectHandle, matrix):
    ret = lib.simSetObjectMatrix(objectHandle, relativeToObjectHandle, matrix)
    _check_return(ret)


def simCheckCollision(entity1Handle, entity2Handle):
    state = lib.simCheckCollision(entity1Handle, entity2Handle)
    _check_return(state)
    return state


def simGetPositionOnPath(pathHandle, relativeDistance):
    position = ffi.new('float[3]')
    ret = lib.simGetPositionOnPath(pathHandle, relativeDistance, position)
    _check_return(ret)
    return list(position)


def simGetOrientationOnPath(pathHandle, relativeDistance):
    orientation = ffi.new('float[3]')
    ret = lib.simGetOrientationOnPath(pathHandle, relativeDistance, orientation)
    _check_return(ret)
    return list(orientation)


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
    if ambient_diffuse is None:
        ambient_diffuse = ffi.NULL
    if specular is None:
        specular = ffi.NULL
    if emission is None:
        emission = ffi.NULL
    handle = lib.simAddDrawingObject(
        objectType, size, duplicateTolerance, parentObjectHandle, maxItemCount,
        ambient_diffuse, ffi.NULL, specular, emission)
    _check_return(handle)
    return handle


def simRemoveDrawingObject(objectHandle):
    ret = lib.simRemoveDrawingObject(objectHandle)
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
        itemData = ffi.NULL
    ret = lib.simAddDrawingObjectItem(objectHandle, itemData)
    _check_return(ret)


def simGetSimulationTimeStep():
    step = lib.simGetSimulationTimeStep()
    _check_return(step)
    return step


def simResetDynamicObject(objectHandle):
    ret = lib.simResetDynamicObject(objectHandle)
    _check_return(ret)


def simGetJointType(objectHandle):
    type = lib.simGetJointType(objectHandle)
    _check_return(type)
    return type


def simRMLPos(dofs, smallestTimeStep, flags, currentPosVelAccel,
              maxVelAccelJerk, selection, targetPosVel):
    smallestTimeStep = ffi.cast('double', smallestTimeStep)
    handle = lib.simRMLPos(dofs, smallestTimeStep, flags, currentPosVelAccel,
                           maxVelAccelJerk, selection, targetPosVel, ffi.NULL)
    _check_return(handle)
    return handle


def simRMLVel(dofs, smallestTimeStep, flags, currentPosVelAccel, maxAccelJerk,
              selection, targetVel):
    handle = lib.simRMLVel(dofs, smallestTimeStep, flags, currentPosVelAccel,
                           maxAccelJerk, selection, targetVel, ffi.NULL)
    _check_return(handle)
    return handle


def simRMLStep(handle, timeStep, dofs):
    newPosVelAccel = ffi.new('double[%d]' % (dofs * 3))
    # timeStep = ffi.cast('double', timeStep)
    state = lib.simRMLStep(handle, timeStep, newPosVelAccel, ffi.NULL, ffi.NULL)
    _check_return(state)
    return state, list(newPosVelAccel)


def simRMLRemove(handle):
    ret = lib.simRMLRemove(handle)
    _check_return(ret)


def simImportMesh(fileformat, pathAndFilename, options,
                  identicalVerticeTolerance, scalingFactor):
    outVerticies = ffi.new('float ***')
    outVerticiesCount = ffi.new('int **')
    outIndices = ffi.new('int ***')
    outIndicesCount = ffi.new('int **')
    outNames = ffi.new('char ***')
    count = lib.simImportMesh(
        fileformat, pathAndFilename.encode('ascii'), options,
        identicalVerticeTolerance, scalingFactor, outVerticies,
        outVerticiesCount, outIndices, outIndicesCount, ffi.NULL, outNames)
    _check_return(count)
    retVerticies = [[outVerticies[0][i][j]
                     for j in range(outVerticiesCount[0][i])]
                    for i in range(count)]
    retIndices = [[outIndices[0][i][j]
                   for j in range(outIndicesCount[0][i])]
                  for i in range(count)]
    retNames = [ffi.string(outNames[0][i]).decode('utf-8')
                for i in range(count)]
    for i in range(count):
        simReleaseBuffer(ffi.cast('char *', outVerticies[0][i]))
        simReleaseBuffer(ffi.cast('char *', outIndices[0][i]))
        simReleaseBuffer(outNames[0][i])
    simReleaseBuffer(ffi.cast('char *', outVerticies[0]))
    simReleaseBuffer(ffi.cast('char *', outVerticiesCount[0]))
    simReleaseBuffer(ffi.cast('char *', outIndices[0]))
    simReleaseBuffer(ffi.cast('char *', outIndicesCount[0]))
    simReleaseBuffer(ffi.cast('char *', outNames[0]))
    return retVerticies, retIndices, retNames


def simImportShape(fileformat, pathAndFilename, options,
                   identicalVerticeTolerance, scalingFactor):
    handle = lib.simImportShape(
        fileformat, pathAndFilename.encode('ascii'), options,
        identicalVerticeTolerance, scalingFactor)
    _check_return(handle)
    return handle


def simCreateMeshShape(options, shadingAngle, vertices, indices):
    ret = lib.simCreateMeshShape(options, shadingAngle, vertices, len(vertices),
                                 indices, len(indices), ffi.NULL)
    return ret


def simGetShapeMesh(shapeHandle):
    outVerticies = ffi.new('float **')
    outVerticiesCount = ffi.new('int *')
    outIndices = ffi.new('int **')
    outIndicesCount = ffi.new('int *')
    # outNormals is 3 times the size of outIndicesCount
    outNormals = ffi.new('float **')

    ret = lib.simGetShapeMesh(shapeHandle, outVerticies, outVerticiesCount,
                              outIndices, outIndicesCount, outNormals)
    _check_return(ret)
    retVerticies = [outVerticies[0][i]
                    for i in range(outVerticiesCount[0])]
    retIndices = [outIndices[0][i]
                    for i in range(outIndicesCount[0])]
    outNormals = [outIndices[0][i]
                  for i in range(outIndicesCount[0] * 3)]

    simReleaseBuffer(ffi.cast('char *', outVerticies[0]))
    simReleaseBuffer(ffi.cast('char *', outIndices[0]))
    simReleaseBuffer(ffi.cast('char *', outNormals[0]))

    return retVerticies, retIndices, outNormals


def simGetShapeViz(shapeHandle, index):
    info = ffi.new('struct SShapeVizInfo *')
    ret = lib.simGetShapeViz(shapeHandle, index, info)
    _check_return(ret)

    vertices = [info.vertices[i] for i in range(info.verticesSize)]
    indices = [info.indices[i] for i in range(info.indicesSize)]
    normals = [info.normals[i] for i in range(info.indicesSize * 3)]
    colors = list(info.colors)
    textureSize = info.textureRes[0] * info.textureRes[1] * 4
    if textureSize == 0:
        texture = []
        textureCoords = []
    else:
        texture = np.frombuffer(
            ffi.buffer(info.texture, textureSize), np.uint8)
        texture = texture.tolist()
        textureCoords = [info.textureCoords[i] for i in
                        range(info.indicesSize * 2)]

    return SShapeVizInfo(
        vertices=vertices,
        indices=indices,
        normals=normals,
        shadingAngle=info.shadingAngle,
        colors=colors,
        texture=texture,
        textureId=info.textureId,
        textureRes=info.textureRes,
        textureCoords=textureCoords,
        textureApplyMode=info.textureApplyMode,
        textureOptions=info.textureOptions,
    )


def simConvexDecompose(shapeHandle, options, intParams, floatParams):
    return lib.simConvexDecompose(shapeHandle, options, intParams, floatParams)


def simGetJointMode(shapeHandle):
    options = ffi.new('int*')
    mode = lib.simGetJointMode(shapeHandle, options)
    _check_return(mode)
    return mode


def simSetJointMode(shapeHandle, mode):
    options = 0
    ret = lib.simSetJointMode(shapeHandle, mode, options)
    _check_return(ret)


def simCreatePath(attributes, intParams, floatParams, color):
    handle = lib.simCreatePath(attributes, intParams, floatParams, color + [0.]*3 + [0.25]*3 + [0.]*3)
    _check_return(handle)
    return handle


def simAddScript(type):
    handle = lib.simAddScript(type)
    _check_return(handle)
    return handle


def simAssociateScriptWithObject(scriptHandle, objectHandle):
    ret = lib.simAssociateScriptWithObject(scriptHandle, objectHandle)
    _check_return(ret)


def simSetScriptText(scriptHandle, scriptText):
    ret = lib.simSetScriptText(scriptHandle, scriptText.encode('ascii'))
    _check_return(ret)


def simGetScriptText(scriptHandle):
    ret = lib.simGetScriptText(scriptHandle)
    ret = ffi.string(ret).decode('utf-8')
    return ret


def simGetScriptAssociatedWithObject(objectHandle):
    ret = lib.simGetScriptAssociatedWithObject(objectHandle)
    return ret


def simApplyTexture(shapeHandle, textureCoordinates, textCoordSize,
                    texture, textureResolution, options):
    ret = lib.simApplyTexture(shapeHandle, textureCoordinates, textCoordSize,
                              texture, textureResolution, options)
    _check_return(ret)
    return ret


def simCreateTexture(fileName, options):
    # The textureID param that is returned from simCreateTexture seems
    # to be incorrect (in regards to calling simGetShapeTextureId on the
    # generated plane).
    handle = lib.simCreateTexture(fileName.encode('ascii'), options, ffi.NULL,
                                  ffi.NULL, ffi.NULL, 0, ffi.NULL, ffi.NULL,
                                  ffi.NULL)
    _check_return(handle)
    return handle


def simSetShapeTexture(shapeHandle, textureId, mappingMode, options, uvScaling,
                       position, orientation):
    if position is None:
        position = ffi.NULL
    if orientation is None:
        orientation = ffi.NULL

    handle = lib.simSetShapeTexture(shapeHandle, textureId, mappingMode,
                                    options, uvScaling, position, orientation)
    _check_return(handle)


def simGetShapeTextureId(objectHandle):
    ret = lib.simGetShapeTextureId(objectHandle)
    _check_return(ret)
    return ret


def simCopyPasteObjects(objectHandles, options):
    handles = ffi.new('int[]', objectHandles)
    ret = lib.simCopyPasteObjects(handles, len(objectHandles), options)
    _check_return(ret)
    return list(handles)


def simHandleIkGroup(ikGroupHandle):
    ret = lib.simHandleIkGroup(ikGroupHandle)
    _check_return(ret)
    return ret


def simCheckIkGroup(ikGroupHandle, jointHandles):
    jointValues = ffi.new('float[%d]' % len(jointHandles))
    ret = lib.simCheckIkGroup(
        ikGroupHandle, len(jointHandles), jointHandles, jointValues, ffi.NULL)
    _check_return(ret)
    return ret, list(jointValues)


def simComputeJacobian(ikGroupHandle, options):
    # Only works when joints that are in IK or hybrid mode
    ret = lib.simComputeJacobian(ikGroupHandle, options, ffi.NULL)
    _check_return(ret)


def simGetIkGroupMatrix(ikGroupHandle, options):
    matrixSize = ffi.new('int[2]')
    ret = lib.simGetIkGroupMatrix(ikGroupHandle, options, matrixSize)
    flatJacobian = [ret[i] for i in range(matrixSize[0] * matrixSize[1])]
    # matrixSize[0] represents the row count of the Jacobian.
    # matrixSize[0] represents the column count of the Jacobian.
    return flatJacobian, list(matrixSize)


def simCheckDistance(entity1Handle, entity2Handle, threshold):
    distanceData = ffi.new('float [7]')
    ret = lib.simCheckDistance(
        entity1Handle, entity2Handle, threshold, distanceData)
    _check_return(ret)
    return list(distanceData)


def simSetExplicitHandling(generalObjectHandle, explicitFlags):
    ret = lib.simSetExplicitHandling(generalObjectHandle, explicitFlags)
    _check_return(ret)


def simGetExplicitHandling(generalObjectHandle):
    flag = lib.simGetExplicitHandling(generalObjectHandle)
    _check_return(flag)
    return flag


def simUngroupShape(shapeHandle):
    count = ffi.new('int*')
    shapes = lib.simUngroupShape(shapeHandle, count)
    _check_null_return(shapes)
    handles = [shapes[i] for i in range(count[0])]
    # simReleaseBuffer(shapes)
    return handles


def simInvertMatrix(matrix):
    c_matrix = ffi.new('float []', matrix)
    ret = lib.simInvertMatrix(c_matrix)
    _check_return(ret)
    return list(c_matrix)


def simMultiplyMatrices(inMatrix1, inMatrix2):
    outMatrix = ffi.new('float []', len(inMatrix1))
    ret = lib.simMultiplyMatrices(inMatrix1, inMatrix2, outMatrix)
    _check_return(ret)
    _check_null_return(outMatrix)
    return list(outMatrix)


def simGetEulerAnglesFromMatrix(rotationMatrix):
    eulerAngles = ffi.new('float [3]')
    ret = lib.simGetEulerAnglesFromMatrix(rotationMatrix, eulerAngles)
    _check_return(ret)
    _check_null_return(eulerAngles)
    return list(eulerAngles)


def simGetSimulationTime():
    time = lib.simGetSimulationTime()
    _check_return(time)
    return time


def simSetIntegerSignal(signalName, signalValue):
    ret = lib.simSetIntegerSignal(signalName.encode('ascii'), signalValue)
    _check_return(ret)


def simGetIntegerSignal(signalName):
    val = ffi.new('int*')
    ret = lib.simGetIntegerSignal(signalName.encode('ascii'), val)
    _check_return(ret)
    return ret, val[0]


def simClearIntegerSignal(signalName):
    ret = lib.simClearIntegerSignal(signalName.encode('ascii'))
    _check_return(ret)
    return ret


def simSetFloatSignal(signalName, signalValue):
    ret = lib.simSetFloatSignal(signalName.encode('ascii'), signalValue)
    _check_return(ret)


def simGetFloatSignal(signalName):
    val = ffi.new('float*')
    ret = lib.simGetFloatSignal(signalName.encode('ascii'), val)
    _check_return(ret)
    return ret, val[0]


def simClearFloatSignal(signalName):
    ret = lib.simClearFloatSignal(signalName.encode('ascii'))
    _check_return(ret)
    return ret


def simSetDoubleSignal(signalName, signalValue):
    ret = lib.simSetDoubleSignal(signalName.encode('ascii'), signalValue)
    _check_return(ret)


def simGetDoubleSignal(signalName):
    val = ffi.new('double*')
    ret = lib.simGetDoubleSignal(signalName.encode('ascii'), val)
    _check_return(ret)
    return ret, val[0]


def simClearDoubleSignal(signalName):
    ret = lib.simClearDoubleSignal(signalName.encode('ascii'))
    _check_return(ret)
    return ret


def simSetStringSignal(signalName, signalValue):
    ret = lib.simSetStringSignal(
        signalName.encode('ascii'), signalValue.encode('ascii'),
        len(signalValue))
    _check_return(ret)


def simGetStringSignal(signalName):
    valLen = ffi.new('int*')
    str_ret = lib.simGetStringSignal(signalName.encode('ascii'), valLen)
    if str_ret == ffi.NULL:
        # No value.
        return 0, None
    val = ffi.string(str_ret[0:valLen[0]]).decode('utf-8')
    simReleaseBuffer(ffi.cast('char *', str_ret))
    return 1, val


def simClearStringSignal(signalName):
    ret = lib.simClearStringSignal(signalName.encode('ascii'))
    _check_return(ret)
    return ret


def simSetUserParameter(objectHandle, parameterName, parameterValue):
    # TODO: currently not used by PyRep.
    # User params functionality missing in CoppeliaSim.
    parameterLength = len(parameterValue)
    ret = lib.simSetUserParameter(
        objectHandle, parameterName.encode('ascii'),
        parameterValue.encode('ascii'), parameterLength)
    _check_return(ret)


def simGetUserParameter(objectHandle, parameterName):
    # TODO: currently not used by PyRep.
    # User params functionality missing in CoppeliaSim.
    parameterLength = ffi.new('int*')
    parameterValue = lib.simGetUserParameter(
        objectHandle, parameterName.encode('ascii'), parameterLength)
    _check_null_return(parameterValue)
    val = ffi.string((parameterValue[0][:parameterLength[0]])).decode('utf-8')
    simReleaseBuffer(ffi.cast('char *', parameterValue))
    return val


def simCreateOctree(voxelSize, options, pointSize):
    ret = lib.simCreateOctree(voxelSize, options, pointSize, ffi.NULL)
    _check_return(ret)
    return ret


def simInsertVoxelsIntoOctree(octreeHandle, options, points, color, tag):
    if color is None:
        color = ffi.NULL
    if tag is None:
        tag = ffi.NULL
    ret = lib.simInsertVoxelsIntoOctree(octreeHandle, options, points,
                                        len(points)//3, color, tag, ffi.NULL)
    _check_return(ret)
    return ret


def simRemoveVoxelsFromOctree(octreeHandle, options, points):
    if points is None:
        points = ffi.NULL
    if points is ffi.NULL:
        pointCount = 0
    else:
        pointCount = len(points)//3
    ret = lib.simRemoveVoxelsFromOctree(octreeHandle, options, points,
                                        pointCount, ffi.NULL)
    _check_return(ret)
    return ret


def simGetOctreeVoxels(octreeHandle):
    pointCountPointer = ffi.new('int *')
    ret = lib.simGetOctreeVoxels(octreeHandle, pointCountPointer, ffi.NULL)
    if ret == ffi.NULL:
        return []
    pointCount = pointCountPointer[0]
    return list(ret[0:pointCount*3])


def simInsertObjectIntoOctree(octreeHandle, objectHandle, options,
                              color, tag):
    if color is None:
        color = ffi.NULL
    ret = lib.simInsertObjectIntoOctree(octreeHandle, objectHandle, options,
                                        color, tag, ffi.NULL)
    _check_return(ret)
    return ret


def simSubtractObjectFromOctree(octreeHandle, objectHandle, options):
    ret = lib.simSubtractObjectFromOctree(octreeHandle, objectHandle, options,
                                        ffi.NULL)
    _check_return(ret)
    return ret


def simCheckOctreePointOccupancy(octreeHandle, options, points):
    ret = lib.simCheckOctreePointOccupancy(octreeHandle, options, points,
                                           len(points)//3, ffi.NULL, ffi.NULL,
                                           ffi.NULL)
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
            contact = ffi.new('float[9]')
            ext = sim_handleflag_extended
        else:
            contact = ffi.new('float[6]')
            ext = 0

        object_handles = ffi.new('int[2]')
        result = lib.simGetContactInfo(sim_handle_all, contact_obj_handle, index + ext, object_handles,
                                       contact)
        contact_info = {
            "contact": list(contact),
            "contact_handles": list(object_handles)
        }
        contact_list.append(contact_info)
        index += 1
    contact_list.pop(-1)  # remove the all zero value
    return contact_list


def simGetConfigForTipPose(ikGroupHandle, jointHandles, thresholdDist, maxTimeInMs, metric, collisionPairs, jointOptions, lowLimits, ranges):
    jointCnt = len(jointHandles)
    collisionPairCnt = len(collisionPairs) // 2
    collisionPairs = ffi.NULL if len(collisionPairs) == 0 else collisionPairs
    retConfigm = ffi.new('float[%d]' % jointCnt)
    reserved = ffi.NULL
    metric = ffi.NULL if metric is None else metric
    jointOptions = ffi.NULL if jointOptions is None else jointOptions
    ret = lib.simGetConfigForTipPose(
        ikGroupHandle, jointCnt, jointHandles, thresholdDist,
        maxTimeInMs, retConfigm, metric, collisionPairCnt, collisionPairs,
        jointOptions, lowLimits, ranges, reserved)
    _check_return(ret)
    _check_null_return(retConfigm)
    return list(retConfigm) if ret == 1 else []


def generateIkPath(ikGroupHandle, jointHandles, ptCnt, collisionPairs, jointOptions):
    jointCnt = len(jointHandles)
    collisionPairCnt = len(collisionPairs) // 2
    collisionPairs = ffi.NULL if len(collisionPairs) == 0 else collisionPairs
    reserved = ffi.NULL
    jointOptions = ffi.NULL if jointOptions is None else jointOptions
    ret = lib.simGenerateIkPath(
        ikGroupHandle, jointCnt, jointHandles, ptCnt, collisionPairCnt,
        collisionPairs, jointOptions, reserved)
    return [] if ret == ffi.NULL else [ret[i] for i in range(ptCnt * jointCnt)]


def simGetDecimatedMesh(inVertices, inIndices, decimationPercent):
    outVerticies = ffi.new('float **')
    outVerticiesCount = ffi.new('int *')
    outIndices = ffi.new('int **')
    outIndicesCount = ffi.new('int *')
    # outNormals is 3 times the size of outIndicesCount
    # outNormals = ffi.new('float **')

    ret = lib.simGetDecimatedMesh(inVertices, len(inVertices),
                                  inIndices, len(inIndices),
                                  outVerticies, outVerticiesCount,
                                  outIndices, outIndicesCount,
                                  decimationPercent, 0, ffi.NULL)
    _check_return(ret)
    retVerticies = [outVerticies[0][i]
                    for i in range(outVerticiesCount[0])]
    retIndices = [outIndices[0][i]
                    for i in range(outIndicesCount[0])]

    simReleaseBuffer(ffi.cast('char *', outVerticies[0]))
    simReleaseBuffer(ffi.cast('char *', outIndices[0]))

    return retVerticies, retIndices


def simComputeMassAndInertia(shapeHandle, density):
    ret = lib.simComputeMassAndInertia(shapeHandle, density)
    _check_return(ret)
    return ret


def simAddForce(shapeHandle, position, force):
    ret = lib.simAddForce(shapeHandle, position, force)
    _check_return(ret)


def simAddForceAndTorque(shapeHandle, force, torque):
    ret = lib.simAddForceAndTorque(shapeHandle,
                                   ffi.NULL if force is None else force,
                                   ffi.NULL if torque is None else torque)
    _check_return(ret)


def simSetLightParameters(shapeHandle, state, diffusePart=None, specularPart=None):
    ret = lib.simSetLightParameters(shapeHandle, state, ffi.NULL,
                                    ffi.NULL if diffusePart is None else diffusePart,
                                    ffi.NULL if specularPart is None else specularPart)
    _check_return(ret)


def simGetLightParameters(shapeHandle):
    diffusePart = ffi.new('float[3]')
    specularPart = ffi.new('float[3]')
    ret = lib.simGetLightParameters(shapeHandle, ffi.NULL, diffusePart, specularPart)
    _check_return(ret)
    return ret, list(diffusePart), list(specularPart)
