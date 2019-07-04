from .vrepConst import *
from ._v_rep_cffi import ffi, lib
import numpy as np


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
    if handle <= 0:
        raise RuntimeError('Handle %s does not exist.' % objectName)
    return handle


def simGetIkGroupHandle(ikGroupName):
    handle = lib.simGetIkGroupHandle(ikGroupName.encode('ascii'))
    if handle <= 0:
        raise RuntimeError('Ik group does not exist.')
    return handle


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


def simGetJointInterval(jointHandle):
    cyclic = ffi.new('char *')
    interval = ffi.new('float [2]')
    ret = lib.simGetJointInterval(jointHandle, cyclic, interval)
    _check_return(ret)
    return ffi.string(cyclic).decode('utf-8') != '', list(interval)


def simSetJointInterval(jointHandle, cyclic, interval):
    ret = lib.simSetJointInterval(jointHandle, cyclic, interval)
    _check_return(ret)


def simBreakForceSensor(forceSensorHandle):
    lib.simBreakForceSensor(forceSensorHandle)


def simReadForceSensor(forceSensorHandle):
    forceVector  = ffi.new('float[3]')
    torqueVector = ffi.new('float[3]')
    state = lib.simReadForceSensor(forceSensorHandle, forceVector, torqueVector)
    return state, list(forceVector), list(torqueVector)


def simReleaseBuffer(pointer):
    lib.simReleaseBuffer(pointer)


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
    # Wrap the buffer with numpy. Take a copy otherwise the data is lost when we release the buffer
    img = np.frombuffer(ffi.buffer(img_buffer, resolution[0]*resolution[1]*3*s ), np_T).copy()
    img = img.reshape(resolution[1], resolution[0], 3)
    img = np.flip(img, 0)  # Image is upside-down
    simReleaseBuffer(ffi.cast('char *', img_buffer))
    return img


def simGetVisionSensorDepthBuffer(sensorHandle, resolution):
    img_buffer = lib.simGetVisionSensorDepthBuffer(sensorHandle)
    T = ffi.getctype(ffi.typeof(img_buffer).item)  # Buffer data type
    s = ffi.sizeof(T)  # datatype size
    np_T = np.dtype('f{:d}'.format(s))  # Numpy equivalent, e.g. float is 'f4'
    # Wrap the buffer with numpy. Take a copy otherwise the data is lost when we release the buffer
    img = np.frombuffer(ffi.buffer(img_buffer, resolution[0]*resolution[1]*s ), np_T).copy()
    img = img.reshape(resolution[1], resolution[0])
    img = np.flip(img, 0)  # Image is upside-down
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


def simGetStringParameter(parameter):
    val = lib.simGetStringParameter(parameter)
    _check_null_return(val)
    sval = ffi.string(val).decode('utf-8')
    simReleaseBuffer(val)
    return sval


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


def simCreatePath(attributes):
    handle = lib.simCreatePath(attributes, ffi.NULL, ffi.NULL, ffi.NULL)
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
