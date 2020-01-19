from cffi import FFI
import os
import glob
from shutil import copyfile

# Get PYREP root and find the needed files to compile the cffi lib.

if 'COPPELIASIM_ROOT' not in os.environ:
    raise RuntimeError('COPPELIASIM_ROOT not defined.')

if not os.path.exists(os.environ['COPPELIASIM_ROOT']):
    raise RuntimeError('COPPELIASIM_ROOT defined, but is not a valid path.')

ffibuilder = FFI()

ffibuilder.cdef("""

// ==============
// simTypes.h
// ==============

// Various types used in the interface functions:
typedef unsigned char simBool;
typedef char simChar;
typedef int simInt;
typedef float simFloat;
typedef double simDouble;
typedef void simVoid;
typedef unsigned char simUChar;
typedef unsigned int simUInt;
typedef unsigned long long int simUInt64;

struct SScriptCallBack
{
    simInt objectID;
    simInt scriptID;
    simInt stackID;
    simChar waitUntilZero;
    simChar* raiseErrorWithMessage;
};

struct SShapeVizInfo
{
    simFloat* vertices;
    simInt verticesSize;
    simInt* indices;
    simInt indicesSize;
    simFloat shadingAngle;
    simFloat* normals;
    simFloat colors[9];
    simChar* texture; /*rgba*/
    simInt textureId;
    simInt textureRes[2];
    simFloat* textureCoords;
    simInt textureApplyMode;
    simInt textureOptions;
};

struct SLuaCallBack
{
    simInt objectID;
    simBool* inputBool;
    simInt* inputInt;
    simFloat* inputFloat;
    simChar* inputChar;
    simInt inputArgCount;
    simInt* inputArgTypeAndSize;
    simBool* outputBool;
    simInt* outputInt;
    simFloat* outputFloat;
    simChar* outputChar;
    simInt outputArgCount;
    simInt* outputArgTypeAndSize;
    simChar waitUntilZero;
    simChar* inputCharBuff;
    simChar* outputCharBuff;
    simInt scriptID;
    simDouble* inputDouble;
    simDouble* outputDouble;
};

typedef int (*contactCallback)(int,int,int,int*,float*);
typedef int (*jointCtrlCallback)(int,int,int,const int*,const float*,float*);


// ==============
// sim.h
// ==============

simInt simRunSimulator(const simChar* applicationName,simInt options,simVoid(*initCallBack)(),simVoid(*loopCallBack)(),simVoid(*deinitCallBack)());
simInt simRunSimulatorEx(const simChar* applicationName,simInt options,simVoid(*initCallBack)(),simVoid(*loopCallBack)(),simVoid(*deinitCallBack)(),simInt stopDelay,const simChar* sceneOrModelToLoad);
simVoid* simGetMainWindow(simInt type);
simChar* simGetLastError();
simInt simSetBooleanParameter(simInt parameter,simBool boolState);
simInt simGetBooleanParameter(simInt parameter);
simInt simSetBoolParameter(simInt parameter,simBool boolState);
simInt simGetBoolParameter(simInt parameter);
simInt simSetIntegerParameter(simInt parameter,simInt intState);
simInt simGetIntegerParameter(simInt parameter,simInt* intState);
simInt simSetInt32Parameter(simInt parameter,simInt intState);
simInt simGetInt32Parameter(simInt parameter,simInt* intState);
simInt simGetUInt64Parameter(simInt parameter,simUInt64* intState);
simInt simSetFloatingParameter(simInt parameter,simFloat floatState);
simInt simGetFloatingParameter(simInt parameter,simFloat* floatState);
simInt simSetFloatParameter(simInt parameter,simFloat floatState);
simInt simGetFloatParameter(simInt parameter,simFloat* floatState);
simInt simSetStringParameter(simInt parameter,const simChar* str);
simChar* simGetStringParameter(simInt parameter);
simInt simGetObjectHandle(const simChar* objectName);
simInt simRemoveObject(simInt objectHandle);
simInt simRemoveModel(simInt objectHandle);
simChar* simGetObjectName(simInt objectHandle);
simInt simGetObjects(simInt index,simInt objectType);
simInt simSetObjectName(simInt objectHandle,const simChar* objectName);
simInt simGetCollectionHandle(const simChar* collectionName);
simInt simRemoveCollection(simInt collectionHandle);
simInt simEmptyCollection(simInt collectionHandle);
simChar* simGetCollectionName(simInt collectionHandle);
simInt simSetCollectionName(simInt collectionHandle,const simChar* collectionName);
simInt simGetObjectMatrix(simInt objectHandle,simInt relativeToObjectHandle,simFloat* matrix);
simInt simSetObjectMatrix(simInt objectHandle,simInt relativeToObjectHandle,const simFloat* matrix);
simInt simGetObjectPosition(simInt objectHandle,simInt relativeToObjectHandle,simFloat* position);
simInt simSetObjectPosition(simInt objectHandle,simInt relativeToObjectHandle,const simFloat* position);
simInt simGetObjectOrientation(simInt objectHandle,simInt relativeToObjectHandle,simFloat* eulerAngles);
simInt simGetObjectQuaternion(simInt objectHandle,simInt relativeToObjectHandle,simFloat* quaternion);
simInt simSetObjectQuaternion(simInt objectHandle,simInt relativeToObjectHandle,const simFloat* quaternion);
simInt simSetObjectOrientation(simInt objectHandle,simInt relativeToObjectHandle,const simFloat* eulerAngles);
simInt simGetJointPosition(simInt objectHandle,simFloat* position);
simInt simSetJointPosition(simInt objectHandle,simFloat position);
simInt simSetJointTargetPosition(simInt objectHandle,simFloat targetPosition);
simInt simGetJointTargetPosition(simInt objectHandle,simFloat* targetPosition);
simInt simSetJointMaxForce(simInt objectHandle,simFloat forceOrTorque);
simInt simGetPathPosition(simInt objectHandle,simFloat* position);
simInt simSetPathPosition(simInt objectHandle,simFloat position);
simInt simGetPathLength(simInt objectHandle,simFloat* length);
simInt simGetJointMatrix(simInt objectHandle,simFloat* matrix);
simInt simSetSphericalJointMatrix(simInt objectHandle,const simFloat* matrix);
simInt simGetJointInterval(simInt objectHandle,simBool* cyclic,simFloat* interval);
simInt simSetJointInterval(simInt objectHandle,simBool cyclic,const simFloat* interval);
simInt simGetObjectParent(simInt objectHandle);
simInt simGetObjectChild(simInt objectHandle,simInt index);
simInt simSetObjectParent(simInt objectHandle,simInt parentObjectHandle,simBool keepInPlace);
simInt simGetObjectType(simInt objectHandle);
simInt simGetJointType(simInt objectHandle);
simInt simBuildIdentityMatrix(simFloat* matrix);
simInt simCopyMatrix(const simFloat* matrixIn,simFloat* matrixOut);
simInt simBuildMatrix(const simFloat* position,const simFloat* eulerAngles,simFloat* matrix);
simInt simBuildMatrixQ(const simFloat* position,const simFloat* quaternion,simFloat* matrix);
simInt simGetEulerAnglesFromMatrix(const simFloat* matrix,simFloat* eulerAngles);
simInt simGetQuaternionFromMatrix(const simFloat* matrix,simFloat* quaternion);
simInt simInvertMatrix(simFloat* matrix);
simInt simMultiplyMatrices(const simFloat* matrixIn1,const simFloat* matrixIn2,simFloat* matrixOut);
simInt simInterpolateMatrices(const simFloat* matrixIn1,const simFloat* matrixIn2,simFloat interpolFactor,simFloat* matrixOut);
simInt simTransformVector(const simFloat* matrix,simFloat* vect);
simInt simReservedCommand(simInt v,simInt w);
simFloat simGetSimulationTime();
simInt simGetSimulationState();
simFloat simGetSystemTime();
simInt simGetSystemTimeInMilliseconds(); // deprecated
simUInt simGetSystemTimeInMs(simInt previousTime);
simInt simLoadScene(const simChar* filename);
simInt simCloseScene();
simInt simSaveScene(const simChar* filename);
simInt simLoadModel(const simChar* filename);
simInt simSaveModel(simInt baseOfModelHandle,const simChar* filename);
simInt simAddStatusbarMessage(const simChar* message);
simChar* simGetSimulatorMessage(simInt* messageID,simInt* auxiliaryData,simInt* returnedDataSize);
simInt simAddModuleMenuEntry(const simChar* entryLabel,simInt itemCount,simInt* itemHandles);
simInt simSetModuleMenuItemState(simInt itemHandle,simInt state,const simChar* label);
simInt simDoesFileExist(const simChar* filename);
simInt simIsObjectInSelection(simInt objectHandle);
simInt simAddObjectToSelection(simInt what,simInt objectHandle);
simInt simRemoveObjectFromSelection(simInt what,simInt objectHandle);
simInt simGetObjectSelectionSize();
simInt simGetObjectLastSelection();
simInt simGetObjectSelection(simInt* objectHandles);
simInt simHandleCollision(simInt collisionObjectHandle);
simInt simReadCollision(simInt collisionObjectHandle);
simInt simHandleDistance(simInt distanceObjectHandle,simFloat* smallestDistance);
simInt simReadDistance(simInt distanceObjectHandle,simFloat* smallestDistance);
simInt simHandleProximitySensor(simInt sensorHandle,simFloat* detectedPoint,simInt* detectedObjectHandle,simFloat* normalVector);
simInt simReadProximitySensor(simInt sensorHandle,simFloat* detectedPoint,simInt* detectedObjectHandle,simFloat* normalVector);
simInt simHandleMill(simInt millHandle,simFloat* removedSurfaceAndVolume);
simInt simHandleIkGroup(simInt ikGroupHandle);
simInt simCheckIkGroup(simInt ikGroupHandle,simInt jointCnt,const simInt* jointHandles,simFloat* jointValues,const simInt* jointOptions);
simInt simHandleDynamics(simFloat deltaTime);
simInt simGetScriptHandle(const simChar* scriptName);
simInt simSetScriptText(simInt scriptHandle,const simChar* scriptText);
const simChar* simGetScriptText(simInt scriptHandle);
simInt simGetScriptProperty(simInt scriptHandle,simInt* scriptProperty,simInt* associatedObjectHandle);
simInt simAssociateScriptWithObject(simInt scriptHandle,simInt associatedObjectHandle);
simInt simGetScript(simInt index);
simInt simGetScriptAssociatedWithObject(simInt objectHandle);
simInt simGetCustomizationScriptAssociatedWithObject(simInt objectHandle);
simInt simGetObjectAssociatedWithScript(simInt scriptHandle);
simChar* simGetScriptName(simInt scriptHandle);
simInt simHandleMainScript();
simInt simResetScript(simInt scriptHandle);
simInt simAddScript(simInt scriptProperty);
simInt simRemoveScript(simInt scriptHandle);
simInt simRefreshDialogs(simInt refreshDegree);
simInt simGetCollisionHandle(const simChar* collisionObjectName);
simInt simGetDistanceHandle(const simChar* distanceObjectName);
simInt simGetIkGroupHandle(const simChar* ikGroupName);
simInt simResetCollision(simInt collisionObjectHandle);
simInt simResetDistance(simInt distanceObjectHandle);
simInt simResetProximitySensor(simInt sensorHandle);
simInt simResetMill(simInt millHandle);
simInt simCheckProximitySensor(simInt sensorHandle,simInt entityHandle,simFloat* detectedPoint);
simInt simCheckProximitySensorEx(simInt sensorHandle,simInt entityHandle,simInt detectionMode,simFloat detectionThreshold,simFloat maxAngle,simFloat* detectedPoint,simInt* detectedObjectHandle,simFloat* normalVector);
simInt simCheckProximitySensorEx2(simInt sensorHandle,simFloat* vertexPointer,simInt itemType,simInt itemCount,simInt detectionMode,simFloat detectionThreshold,simFloat maxAngle,simFloat* detectedPoint,simFloat* normalVector);
simChar* simCreateBuffer(simInt size);
simInt simReleaseBuffer(const simChar* buffer);
simInt simCheckCollision(simInt entity1Handle,simInt entity2Handle);
simInt simCheckCollisionEx(simInt entity1Handle,simInt entity2Handle,simFloat** intersectionSegments);
simInt simCheckDistance(simInt entity1Handle,simInt entity2Handle,simFloat threshold,simFloat* distanceData);
simChar* simGetObjectConfiguration(simInt objectHandle);
simInt simSetObjectConfiguration(const simChar* data);
simChar* simGetConfigurationTree(simInt objectHandle);
simInt simSetConfigurationTree(const simChar* data);
simInt simSetSimulationTimeStep(simFloat timeStep);
simFloat simGetSimulationTimeStep();
simInt simGetRealTimeSimulation();
simInt simIsRealTimeSimulationStepNeeded();
simInt simAdjustRealTimeTimer(simInt instanceIndex,simFloat deltaTime);
simInt simGetSimulationPassesPerRenderingPass();
simInt simAdvanceSimulationByOneStep();
simInt simStartSimulation();
simInt simStopSimulation();
simInt simPauseSimulation();
simInt simLoadModule(const simChar* filenameAndPath,const simChar* pluginName);
simInt simUnloadModule(simInt pluginhandle);
simVoid* simSendModuleMessage(simInt message,simInt* auxiliaryData,simVoid* customData,simInt* replyData);
simVoid* simBroadcastMessage(simInt* auxiliaryData,simVoid* customData,simInt* replyData);
simChar* simGetModuleName(simInt index,simUChar* moduleVersion);
simInt simFloatingViewAdd(simFloat posX,simFloat posY,simFloat sizeX,simFloat sizeY,simInt options);
simInt simFloatingViewRemove(simInt floatingViewHandle);
simInt simAdjustView(simInt viewHandleOrIndex,simInt associatedViewableObjectHandle,simInt options,const simChar* viewLabel);
simInt simSetLastError(const simChar* funcName,const simChar* errorMessage);
simInt simHandleGraph(simInt graphHandle,simFloat simulationTime);
simInt simResetGraph(simInt graphHandle);
simInt simSetNavigationMode(simInt navigationMode);
simInt simGetNavigationMode();
simInt simSetPage(simInt index);
simInt simGetPage();
simInt simDisplayDialog(const simChar* titleText,const simChar* mainText,simInt dialogType,const simChar* initialText,const simFloat* titleColors,const simFloat* dialogColors,simInt* elementHandle);
simInt simGetDialogResult(simInt genericDialogHandle);
simChar* simGetDialogInput(simInt genericDialogHandle);
simInt simEndDialog(simInt genericDialogHandle);
simInt simRegisterScriptCallbackFunction(const simChar* funcNameAtPluginName,const simChar* callTips,simVoid(*callBack)(struct SScriptCallBack* cb));
simInt simRegisterScriptVariable(const simChar* varNameAtPluginName,const simChar* varValue,simInt stackHandle);
simInt simSetJointTargetVelocity(simInt objectHandle,simFloat targetVelocity);
simInt simGetJointTargetVelocity(simInt objectHandle,simFloat* targetVelocity);
simInt simSetPathTargetNominalVelocity(simInt objectHandle,simFloat targetNominalVelocity);
simChar* simGetScriptRawBuffer(simInt scriptHandle,simInt bufferHandle);
simInt simSetScriptRawBuffer(simInt scriptHandle,const simChar* buffer,simInt bufferSize);
simInt simReleaseScriptRawBuffer(simInt scriptHandle,simInt bufferHandle);
simInt simCopyPasteObjects(simInt* objectHandles,simInt objectCount,simInt options);
simInt simScaleSelectedObjects(simFloat scalingFactor,simBool scalePositionsToo);
simInt simScaleObjects(const simInt* objectHandles,simInt objectCount,simFloat scalingFactor,simBool scalePositionsToo);
simInt simDeleteSelectedObjects();
simInt simGetObjectUniqueIdentifier(simInt objectHandle,simInt* uniqueIdentifier);
simInt simGetNameSuffix(const simChar* name);
simInt simSendData(simInt targetID,simInt dataHeader,const simChar* dataName,const simChar* data,simInt dataLength,simInt antennaHandle,simFloat actionRadius,simFloat emissionAngle1,simFloat emissionAngle2,simFloat persistence);
simChar* simReceiveData(simInt dataHeader,const simChar* dataName,simInt antennaHandle,simInt index,simInt* dataLength,simInt* senderID,simInt* dataHeaderR,simChar** dataNameR);
simInt simSetGraphUserData(simInt graphHandle,const simChar* dataStreamName,simFloat data);
simInt simSetNameSuffix(simInt nameSuffixNumber);
simInt simAddDrawingObject(simInt objectType,simFloat size,simFloat duplicateTolerance,simInt parentObjectHandle,simInt maxItemCount,const simFloat* ambient_diffuse,const simFloat* setToNULL,const simFloat* specular,const simFloat* emission);
simInt simRemoveDrawingObject(simInt objectHandle);
simInt simAddDrawingObjectItem(simInt objectHandle,const simFloat* itemData);
simInt simAddParticleObject(simInt objectType,simFloat size,simFloat density,const simVoid* params,simFloat lifeTime,simInt maxItemCount,const simFloat* ambient_diffuse,const simFloat* setToNULL,const simFloat* specular,const simFloat* emission);
simInt simRemoveParticleObject(simInt objectHandle);
simInt simAddParticleObjectItem(simInt objectHandle,const simFloat* itemData);
simFloat simGetObjectSizeFactor(simInt objectHandle);
simInt simAnnounceSceneContentChange();
simInt simResetMilling(simInt objectHandle);
simInt simApplyMilling(simInt objectHandle);
simInt simSetIntegerSignal(const simChar* signalName,simInt signalValue);
simInt simGetIntegerSignal(const simChar* signalName,simInt* signalValue);
simInt simClearIntegerSignal(const simChar* signalName);
simInt simSetFloatSignal(const simChar* signalName,simFloat signalValue);
simInt simGetFloatSignal(const simChar* signalName,simFloat* signalValue);
simInt simClearFloatSignal(const simChar* signalName);
simInt simSetDoubleSignal(const simChar* signalName,simDouble signalValue);
simInt simGetDoubleSignal(const simChar* signalName,simDouble* signalValue);
simInt simClearDoubleSignal(const simChar* signalName);
simInt simSetStringSignal(const simChar* signalName,const simChar* signalValue,simInt stringLength);
simChar* simGetStringSignal(const simChar* signalName,simInt* stringLength);
simInt simClearStringSignal(const simChar* signalName);
simChar* simGetSignalName(simInt signalIndex,simInt signalType);
simInt simSetObjectProperty(simInt objectHandle,simInt prop);
simInt simGetObjectProperty(simInt objectHandle);
simInt simSetObjectSpecialProperty(simInt objectHandle,simInt prop);
simInt simGetObjectSpecialProperty(simInt objectHandle);
simInt simGetPositionOnPath(simInt pathHandle,simFloat relativeDistance,simFloat* position);
simInt simGetDataOnPath(simInt pathHandle,simFloat relativeDistance,simInt dataType,simInt* intData,simFloat* floatData);
simInt simGetOrientationOnPath(simInt pathHandle,simFloat relativeDistance,simFloat* eulerAngles);
simInt simGetClosestPositionOnPath(simInt pathHandle,simFloat* absolutePosition,simFloat* pathPosition);
simInt simReadForceSensor(simInt objectHandle,simFloat* forceVector,simFloat* torqueVector);
simInt simBreakForceSensor(simInt objectHandle);
simInt simGetShapeVertex(simInt shapeHandle,simInt groupElementIndex,simInt vertexIndex,simFloat* relativePosition);
simInt simGetShapeTriangle(simInt shapeHandle,simInt groupElementIndex,simInt triangleIndex,simInt* vertexIndices,simFloat* triangleNormals);
simInt simSetLightParameters(simInt objectHandle,simInt state,const simFloat* setToNULL,const simFloat* diffusePart,const simFloat* specularPart);
simInt simGetLightParameters(simInt objectHandle,simFloat* setToNULL,simFloat* diffusePart,simFloat* specularPart);
simInt simGetVelocity(simInt shapeHandle,simFloat* linearVelocity,simFloat* angularVelocity);
simInt simGetObjectVelocity(simInt objectHandle,simFloat* linearVelocity,simFloat* angularVelocity);
simInt simAddForceAndTorque(simInt shapeHandle,const simFloat* force,const simFloat* torque);
simInt simAddForce(simInt shapeHandle,const simFloat* position,const simFloat* force);
simInt simSetExplicitHandling(simInt generalObjectHandle,int explicitFlags);
simInt simGetExplicitHandling(simInt generalObjectHandle);
simInt simGetLinkDummy(simInt dummyHandle);
simInt simSetLinkDummy(simInt dummyHandle,simInt linkedDummyHandle);
simInt simSetModelProperty(simInt objectHandle,simInt modelProperty);
simInt simGetModelProperty(simInt objectHandle);
simInt simSetShapeColor(simInt shapeHandle,const simChar* colorName,simInt colorComponent,const simFloat* rgbData);
simInt simGetShapeColor(simInt shapeHandle,const simChar* colorName,simInt colorComponent,simFloat* rgbData);
simInt simResetDynamicObject(simInt objectHandle);
simInt simSetJointMode(simInt jointHandle,simInt jointMode,simInt options);
simInt simGetJointMode(simInt jointHandle,simInt* options);
simInt simSerialOpen(const simChar* portString,simInt baudRate,simVoid* reserved1,simVoid* reserved2);
simInt simSerialClose(simInt portHandle);
simInt simSerialSend(simInt portHandle,const simChar* data,simInt dataLength);
simInt simSerialRead(simInt portHandle,simChar* buffer,simInt dataLengthToRead);
simInt simSerialCheck(simInt portHandle);
simInt simGetContactInfo(simInt dynamicPass,simInt objectHandle,simInt index,simInt* objectHandles,simFloat* contactInfo);
simInt simSetThreadIsFree(simBool freeMode);
simInt simTubeOpen(simInt dataHeader,const simChar* dataName,simInt readBufferSize,simBool notUsedButKeepZero);
simInt simTubeClose(simInt tubeHandle);
simInt simTubeWrite(simInt tubeHandle,const simChar* data,simInt dataLength);
simChar* simTubeRead(simInt tubeHandle,simInt* dataLength);
simInt simTubeStatus(simInt tubeHandle,simInt* readPacketsCount,simInt* writePacketsCount);
simInt simAuxiliaryConsoleOpen(const simChar* title,simInt maxLines,simInt mode,const simInt* position,const simInt* size,const simFloat* textColor,const simFloat* backgroundColor);
simInt simAuxiliaryConsoleClose(simInt consoleHandle);
simInt simAuxiliaryConsoleShow(simInt consoleHandle,simBool showState);
simInt simAuxiliaryConsolePrint(simInt consoleHandle,const simChar* text);
simInt simImportShape(simInt fileformat,const simChar* pathAndFilename,simInt options,simFloat identicalVerticeTolerance,simFloat scalingFactor);
simInt simImportMesh(simInt fileformat,const simChar* pathAndFilename,simInt options,simFloat identicalVerticeTolerance,simFloat scalingFactor,simFloat*** vertices,simInt** verticesSizes,simInt*** indices,simInt** indicesSizes,simFloat*** reserved,simChar*** names);
simInt simExportMesh(simInt fileformat,const simChar* pathAndFilename,simInt options,simFloat scalingFactor,simInt elementCount,const simFloat** vertices,const simInt* verticesSizes,const simInt** indices,const simInt* indicesSizes,simFloat** reserved,const simChar** names);
simInt simCreateMeshShape(simInt options,simFloat shadingAngle,const simFloat* vertices,simInt verticesSize,const simInt* indices,simInt indicesSize,simFloat* reserved);
simInt simCreatePureShape(simInt primitiveType,simInt options,const simFloat* sizes,simFloat mass,const simInt* precision);
simInt simCreateHeightfieldShape(simInt options,simFloat shadingAngle,simInt xPointCount,simInt yPointCount,simFloat xSize,const simFloat* heights);
simInt simGetShapeMesh(simInt shapeHandle,simFloat** vertices,simInt* verticesSize,simInt** indices,simInt* indicesSize,simFloat** normals);
simInt simAddBanner(const simChar* label,simFloat size,simInt options,const simFloat* positionAndEulerAngles,simInt parentObjectHandle,const simFloat* labelColors,const simFloat* backgroundColors);
simInt simRemoveBanner(simInt bannerID);
simInt simCreateJoint(simInt jointType,simInt jointMode,simInt options,const simFloat* sizes,const simFloat* colorA,const simFloat* colorB);
simInt simGetObjectIntParameter(simInt objectHandle,simInt parameterID,simInt* parameter);
simInt simSetObjectIntParameter(simInt objectHandle,simInt parameterID,simInt parameter);
simInt simGetObjectInt32Parameter(simInt objectHandle,simInt parameterID,simInt* parameter);
simInt simSetObjectInt32Parameter(simInt objectHandle,simInt parameterID,simInt parameter);
simInt simGetObjectFloatParameter(simInt objectHandle,simInt parameterID,simFloat* parameter);
simInt simSetObjectFloatParameter(simInt objectHandle,simInt parameterID,simFloat parameter);
simChar* simGetObjectStringParameter(simInt objectHandle,simInt parameterID,simInt* parameterLength);
simInt simSetObjectStringParameter(simInt objectHandle,simInt parameterID,simChar* parameter,simInt parameterLength);
simInt simSetSimulationPassesPerRenderingPass(simInt p);
simInt simGetRotationAxis(const simFloat* matrixStart,const simFloat* matrixGoal,simFloat* axis,simFloat* angle);
simInt simRotateAroundAxis(const simFloat* matrixIn,const simFloat* axis,const simFloat* axisPos,simFloat angle,simFloat* matrixOut);
simInt simGetJointForce(simInt jointHandle,simFloat* forceOrTorque);
simInt simGetJointMaxForce(simInt jointHandle,simFloat* forceOrTorque);
simInt simSetArrayParameter(simInt parameter,const simVoid* arrayOfValues);
simInt simGetArrayParameter(simInt parameter,simVoid* arrayOfValues);
simInt simSetIkGroupProperties(simInt ikGroupHandle,simInt resolutionMethod,simInt maxIterations,simFloat damping,void* reserved);
simInt simSetIkElementProperties(simInt ikGroupHandle,simInt tipDummyHandle,simInt constraints,const simFloat* precision,const simFloat* weight,void* reserved);
simInt simCameraFitToView(simInt viewHandleOrIndex,simInt objectCount,const simInt* objectHandles,simInt options,simFloat scaling);
simInt simPersistentDataWrite(const simChar* dataName,const simChar* dataValue,simInt dataLength,simInt options);
simChar* simPersistentDataRead(const simChar* dataName,simInt* dataLength);
simInt simIsHandleValid(simInt generalObjectHandle,simInt generalObjectType);
simInt simHandleVisionSensor(simInt visionSensorHandle,simFloat** auxValues,simInt** auxValuesCount);
simInt simReadVisionSensor(simInt visionSensorHandle,simFloat** auxValues,simInt** auxValuesCount);
simInt simResetVisionSensor(simInt visionSensorHandle);
simInt simCheckVisionSensor(simInt visionSensorHandle,simInt entityHandle,simFloat** auxValues,simInt** auxValuesCount);
simFloat* simCheckVisionSensorEx(simInt visionSensorHandle,simInt entityHandle,simBool returnImage);
simInt simGetVisionSensorResolution(simInt visionSensorHandle,simInt* resolution);
simFloat* simGetVisionSensorImage(simInt visionSensorHandle);
simUChar* simGetVisionSensorCharImage(simInt visionSensorHandle,simInt* resolutionX,simInt* resolutionY);
simInt simSetVisionSensorImage(simInt visionSensorHandle,const simFloat* image);
simInt simSetVisionSensorCharImage(simInt visionSensorHandle,const simUChar* image);
simFloat* simGetVisionSensorDepthBuffer(simInt visionSensorHandle);
simInt simRMLPosition(simInt dofs,simDouble timeStep,simInt flags,const simDouble* currentPosVelAccel,const simDouble* maxVelAccelJerk,const simBool* selection,const simDouble* targetPosVel,simDouble* newPosVelAccel,simVoid* auxData);
simInt simRMLVelocity(simInt dofs,simDouble timeStep,simInt flags,const simDouble* currentPosVelAccel,const simDouble* maxAccelJerk,const simBool* selection,const simDouble* targetVel,simDouble* newPosVelAccel,simVoid* auxData);
simInt simRMLPos(simInt dofs,simDouble smallestTimeStep,simInt flags,const simDouble* currentPosVelAccel,const simDouble* maxVelAccelJerk,const simBool* selection,const simDouble* targetPosVel,simVoid* auxData);
simInt simRMLVel(simInt dofs,simDouble smallestTimeStep,simInt flags,const simDouble* currentPosVelAccel,const simDouble* maxAccelJerk,const simBool* selection,const simDouble* targetVel,simVoid* auxData);
simInt simRMLStep(simInt handle,simDouble timeStep,simDouble* newPosVelAccel,simVoid* auxData,simVoid* reserved);
simInt simRMLRemove(simInt handle);
simChar* simFileDialog(simInt mode,const simChar* title,const simChar* startPath,const simChar* initName,const simChar* extName,const simChar* ext);
simInt simMsgBox(simInt dlgType,simInt buttons,const simChar* title,const simChar* message);
simInt simCreateDummy(simFloat size,const simFloat* color);
simInt simSetShapeMassAndInertia(simInt shapeHandle,simFloat mass,const simFloat* inertiaMatrix,const simFloat* centerOfMass,const simFloat* transformation);
simInt simGetShapeMassAndInertia(simInt shapeHandle,simFloat* mass,simFloat* inertiaMatrix,simFloat* centerOfMass,const simFloat* transformation);
simInt simGroupShapes(const simInt* shapeHandles,simInt shapeCount);
simInt* simUngroupShape(simInt shapeHandle,simInt* shapeCount);
simInt simCreateProximitySensor(simInt sensorType,simInt subType,simInt options,const simInt* intParams,const simFloat* floatParams,const simFloat* color);
simInt simCreateForceSensor(simInt options,const simInt* intParams,const simFloat* floatParams,const simFloat* color);
simInt simCreateVisionSensor(simInt options,const simInt* intParams,const simFloat* floatParams,const simFloat* color);
simInt simConvexDecompose(simInt shapeHandle,simInt options,const simInt* intParams,const simFloat* floatParams);
simInt simCreatePath(simInt attributes,const simInt* intParams,const simFloat* floatParams,const simFloat* color);
simInt simInsertPathCtrlPoints(simInt pathHandle,simInt options,simInt startIndex,simInt ptCnt,const simVoid* ptData);
simInt simCutPathCtrlPoints(simInt pathHandle,simInt startIndex,simInt ptCnt);
simFloat* simGetIkGroupMatrix(simInt ikGroupHandle,simInt options,simInt* matrixSize);
simInt simAddGhost(simInt ghostGroup,simInt objectHandle,simInt options,simFloat startTime,simFloat endTime,const simFloat* color);
simInt simModifyGhost(simInt ghostGroup,simInt ghostId,simInt operation,simFloat floatValue,simInt options,simInt optionsMask,const simFloat* colorOrTransformation);
simVoid simQuitSimulator(simBool ignoredArgument);
simInt simGetThreadId();
simInt simLockResources(simInt lockType,simInt reserved);
simInt simUnlockResources(simInt lockHandle);
simInt simEnableEventCallback(simInt eventCallbackType,const simChar* plugin,simInt reserved);
simInt simSetShapeMaterial(simInt shapeHandle,simInt materialIdOrShapeHandle);
simInt simGetTextureId(const simChar* textureName,simInt* resolution);
simChar* simReadTexture(simInt textureId,simInt options,simInt posX,simInt posY,simInt sizeX,simInt sizeY);
simInt simWriteTexture(simInt textureId,simInt options,const simChar* data,simInt posX,simInt posY,simInt sizeX,simInt sizeY,simFloat interpol);
simInt simCreateTexture(const simChar* fileName,simInt options,const simFloat* planeSizes,const simFloat* scalingUV,const simFloat* xy_g,simInt fixedResolution,simInt* textureId,simInt* resolution,const simVoid* reserved);
simInt simWriteCustomDataBlock(simInt objectHandle,const simChar* tagName,const simChar* data,simInt dataSize);
simChar* simReadCustomDataBlock(simInt objectHandle,const simChar* tagName,simInt* dataSize);
simChar* simReadCustomDataBlockTags(simInt objectHandle,simInt* tagCount);
simInt simAddPointCloud(simInt pageMask,simInt layerMask,simInt objectHandle,simInt options,simFloat pointSize,simInt ptCnt,const simFloat* pointCoordinates,const simChar* defaultColors,const simChar* pointColors,const simFloat* pointNormals);
simInt simModifyPointCloud(simInt pointCloudHandle,simInt operation,const simInt* intParam,const simFloat* floatParam);
simInt simGetShapeGeomInfo(simInt shapeHandle,simInt* intData,simFloat* floatData,simVoid* reserved);
simInt* simGetObjectsInTree(simInt treeBaseHandle,simInt objectType,simInt options,simInt* objectCount);
simInt simSetObjectSizeValues(simInt objectHandle,const simFloat* sizeValues);
simInt simGetObjectSizeValues(simInt objectHandle,simFloat* sizeValues);
simInt simScaleObject(simInt objectHandle,simFloat xScale,simFloat yScale,simFloat zScale,simInt options);
simInt simSetShapeTexture(simInt shapeHandle,simInt textureId,simInt mappingMode,simInt options,const simFloat* uvScaling,const simFloat* position,const simFloat* orientation);
simInt simGetShapeTextureId(simInt shapeHandle);
simInt* simGetCollectionObjects(simInt collectionHandle,simInt* objectCount);
simInt simSetScriptAttribute(simInt scriptHandle,simInt attributeID,simFloat floatVal,simInt intOrBoolVal);
simInt simGetScriptAttribute(simInt scriptHandle,simInt attributeID,simFloat* floatVal,simInt* intOrBoolVal);
simInt simReorientShapeBoundingBox(simInt shapeHandle,simInt relativeToHandle,simInt reservedSetToZero);
simInt simSwitchThread();
simInt simCreateIkGroup(simInt options,const simInt* intParams,const simFloat* floatParams,const simVoid* reserved);
simInt simRemoveIkGroup(simInt ikGroupHandle);
simInt simCreateIkElement(simInt ikGroupHandle,simInt options,const simInt* intParams,const simFloat* floatParams,const simVoid* reserved);
simInt simCreateCollection(const simChar* collectionName,simInt options);
simInt simAddObjectToCollection(simInt collectionHandle,simInt objectHandle,simInt what,simInt options);
simInt simSaveImage(const simUChar* image,const simInt* resolution,simInt options,const simChar* filename,simInt quality,simVoid* reserved);
simUChar* simLoadImage(simInt* resolution,simInt options,const simChar* filename,simVoid* reserved);
simUChar* simGetScaledImage(const simUChar* imageIn,const simInt* resolutionIn,simInt* resolutionOut,simInt options,simVoid* reserved);
simInt simTransformImage(simUChar* image,const simInt* resolution,simInt options,const simFloat* floatParams,const simInt* intParams,simVoid* reserved);
simInt simGetQHull(const simFloat* inVertices,simInt inVerticesL,simFloat** verticesOut,simInt* verticesOutL,simInt** indicesOut,simInt* indicesOutL,simInt reserved1,const simFloat* reserved2);
simInt simGetDecimatedMesh(const simFloat* inVertices,simInt inVerticesL,const simInt* inIndices,simInt inIndicesL,simFloat** verticesOut,simInt* verticesOutL,simInt** indicesOut,simInt* indicesOutL,simFloat decimationPercent,simInt reserved1,const simFloat* reserved2);
simInt simExportIk(const simChar* pathAndFilename,simInt reserved1,simVoid* reserved2);
simInt simCallScriptFunctionEx(simInt scriptHandleOrType,const simChar* functionNameAtScriptName,simInt stackId);
simInt simComputeJacobian(simInt ikGroupHandle,simInt options,simVoid* reserved);
simInt simGetConfigForTipPose(simInt ikGroupHandle,simInt jointCnt,const simInt* jointHandles,simFloat thresholdDist,simInt maxTimeInMs,simFloat* retConfig,const simFloat* metric,simInt collisionPairCnt,const simInt* collisionPairs,const simInt* jointOptions,const simFloat* lowLimits,const simFloat* ranges,simVoid* reserved);
simFloat* simGenerateIkPath(simInt ikGroupHandle,simInt jointCnt,const simInt* jointHandles,simInt ptCnt,simInt collisionPairCnt,const simInt* collisionPairs,const simInt* jointOptions,simVoid* reserved);
simChar* simGetExtensionString(simInt objectHandle,simInt index,const char* key);
simInt simComputeMassAndInertia(simInt shapeHandle,simFloat density);
simInt simCreateStack();
simInt simReleaseStack(simInt stackHandle);
simInt simCopyStack(simInt stackHandle);
simInt simPushNullOntoStack(simInt stackHandle);
simInt simPushBoolOntoStack(simInt stackHandle,simBool value);
simInt simPushInt32OntoStack(simInt stackHandle,simInt value);
simInt simPushFloatOntoStack(simInt stackHandle,simFloat value);
simInt simPushDoubleOntoStack(simInt stackHandle,simDouble value);
simInt simPushStringOntoStack(simInt stackHandle,const simChar* value,simInt stringSize);
simInt simPushUInt8TableOntoStack(simInt stackHandle,const simUChar* values,simInt valueCnt);
simInt simPushInt32TableOntoStack(simInt stackHandle,const simInt* values,simInt valueCnt);
simInt simPushFloatTableOntoStack(simInt stackHandle,const simFloat* values,simInt valueCnt);
simInt simPushDoubleTableOntoStack(simInt stackHandle,const simDouble* values,simInt valueCnt);
simInt simPushTableOntoStack(simInt stackHandle);
simInt simInsertDataIntoStackTable(simInt stackHandle);
simInt simGetStackSize(simInt stackHandle);
simInt simPopStackItem(simInt stackHandle,simInt count);
simInt simMoveStackItemToTop(simInt stackHandle,simInt cIndex);
simInt simIsStackValueNull(simInt stackHandle);
simInt simGetStackBoolValue(simInt stackHandle,simBool* boolValue);
simInt simGetStackInt32Value(simInt stackHandle,simInt* numberValue);
simInt simGetStackFloatValue(simInt stackHandle,simFloat* numberValue);
simInt simGetStackDoubleValue(simInt stackHandle,simDouble* numberValue);
simChar* simGetStackStringValue(simInt stackHandle,simInt* stringSize);
simInt simGetStackTableInfo(simInt stackHandle,simInt infoType);
simInt simGetStackUInt8Table(simInt stackHandle,simUChar* array,simInt count);
simInt simGetStackInt32Table(simInt stackHandle,simInt* array,simInt count);
simInt simGetStackFloatTable(simInt stackHandle,simFloat* array,simInt count);
simInt simGetStackDoubleTable(simInt stackHandle,simDouble* array,simInt count);
simInt simUnfoldStackTable(simInt stackHandle);
simInt simDebugStack(simInt stackHandle,simInt cIndex);
simInt simSetScriptVariable(simInt scriptHandleOrType,const simChar* variableNameAtScriptName,simInt stackHandle);
simFloat simGetEngineFloatParameter(simInt paramId,simInt objectHandle,const simVoid* object,simBool* ok);
simInt simGetEngineInt32Parameter(simInt paramId,simInt objectHandle,const simVoid* object,simBool* ok);
simBool simGetEngineBoolParameter(simInt paramId,simInt objectHandle,const simVoid* object,simBool* ok);
simInt simSetEngineFloatParameter(simInt paramId,simInt objectHandle,const simVoid* object,simFloat val);
simInt simSetEngineInt32Parameter(simInt paramId,simInt objectHandle,const simVoid* object,simInt val);
simInt simSetEngineBoolParameter(simInt paramId,simInt objectHandle,const simVoid* object,simBool val);
simInt simCreateOctree(simFloat voxelSize,simInt options,simFloat pointSize,simVoid* reserved);
simInt simCreatePointCloud(simFloat maxVoxelSize,simInt maxPtCntPerVoxel,simInt options,simFloat pointSize,simVoid* reserved);
simInt simSetPointCloudOptions(simInt pointCloudHandle,simFloat maxVoxelSize,simInt maxPtCntPerVoxel,simInt options,simFloat pointSize,simVoid* reserved);
simInt simGetPointCloudOptions(simInt pointCloudHandle,simFloat* maxVoxelSize,simInt* maxPtCntPerVoxel,simInt* options,simFloat* pointSize,simVoid* reserved);
simInt simInsertVoxelsIntoOctree(simInt octreeHandle,simInt options,const simFloat* pts,simInt ptCnt,const simUChar* color,const simUInt* tag,simVoid* reserved);
simInt simRemoveVoxelsFromOctree(simInt octreeHandle,simInt options,const simFloat* pts,simInt ptCnt,simVoid* reserved);
simInt simInsertPointsIntoPointCloud(simInt pointCloudHandle,simInt options,const simFloat* pts,simInt ptCnt,const simUChar* color,simVoid* optionalValues);
simInt simRemovePointsFromPointCloud(simInt pointCloudHandle,simInt options,const simFloat* pts,simInt ptCnt,simFloat tolerance,simVoid* reserved);
simInt simIntersectPointsWithPointCloud(simInt pointCloudHandle,simInt options,const simFloat* pts,simInt ptCnt,simFloat tolerance,simVoid* reserved);
const float* simGetOctreeVoxels(simInt octreeHandle,simInt* ptCnt,simVoid* reserved);
const float* simGetPointCloudPoints(simInt pointCloudHandle,simInt* ptCnt,simVoid* reserved);
simInt simInsertObjectIntoOctree(simInt octreeHandle,simInt objectHandle,simInt options,const simUChar* color,simUInt tag,simVoid* reserved);
simInt simSubtractObjectFromOctree(simInt octreeHandle,simInt objectHandle,simInt options,simVoid* reserved);
simInt simInsertObjectIntoPointCloud(simInt pointCloudHandle,simInt objectHandle,simInt options,simFloat gridSize,const simUChar* color,simVoid* optionalValues);
simInt simSubtractObjectFromPointCloud(simInt pointCloudHandle,simInt objectHandle,simInt options,simFloat tolerance,simVoid* reserved);
simInt simCheckOctreePointOccupancy(simInt octreeHandle,simInt options,const simFloat* points,simInt ptCnt,simUInt* tag,simUInt64* location,simVoid* reserved);
simChar* simOpenTextEditor(const simChar* initText,const simChar* xml,simInt* various);
simChar* simPackTable(simInt stackHandle,simInt* bufferSize);
simInt simUnpackTable(simInt stackHandle,const simChar* buffer,simInt bufferSize);
simInt simSetReferencedHandles(simInt objectHandle,simInt count,const simInt* referencedHandles,const simInt* reserved1,const simInt* reserved2);
simInt simGetReferencedHandles(simInt objectHandle,simInt** referencedHandles,simInt** reserved1,simInt** reserved2);
simInt simGetShapeViz(simInt shapeHandle,simInt index,struct SShapeVizInfo* info);
simInt simExecuteScriptString(simInt scriptHandleOrType,const simChar* stringAtScriptName,simInt stackHandle);
simChar* simGetApiFunc(simInt scriptHandleOrType,const simChar* apiWord);
simChar* simGetApiInfo(simInt scriptHandleOrType,const simChar* apiWord);
simInt simSetModuleInfo(const simChar* moduleName,simInt infoType,const simChar* stringInfo,simInt intInfo);
simInt simGetModuleInfo(const simChar* moduleName,simInt infoType,simChar** stringInfo,simInt* intInfo);
simInt simIsDeprecated(const simChar* funcOrConst);
simChar* simGetPersistentDataTags(simInt* tagCount);
simInt simEventNotification(const simChar* event);
simInt simApplyTexture(simInt shapeHandle,const simFloat* textureCoordinates,simInt textCoordSize,const simUChar* texture,const simInt* textureResolution,simInt options);
simInt simSetJointDependency(simInt jointHandle,simInt masterJointHandle,simFloat offset,simFloat coeff);
simInt simSetStringNamedParam(const simChar* paramName,const simChar* stringParam,simInt paramLength);
simChar* simGetStringNamedParam(const simChar* paramName,simInt* paramLength);
simChar* simGetUserParameter(simInt objectHandle,const simChar* parameterName,simInt* parameterLength);
simInt simSetUserParameter(simInt objectHandle,const simChar* parameterName,const simChar* parameterValue,simInt parameterLength);



simInt _simGetContactCallbackCount();
const void* _simGetContactCallback(simInt index);
simVoid _simSetDynamicSimulationIconCode(simVoid* object,simInt code);
simVoid _simSetDynamicObjectFlagForVisualization(simVoid* object,simInt flag);
simInt _simGetObjectListSize(simInt objType);
const simVoid* _simGetObjectFromIndex(simInt objType,simInt index);
simInt _simGetObjectID(const simVoid* object);
simInt _simGetObjectType(const simVoid* object);
const simVoid** _simGetObjectChildren(const simVoid* object,simInt* count);
const simVoid* _simGetGeomProxyFromShape(const simVoid* shape);
const simVoid* _simGetParentObject(const simVoid* object);
const simVoid* _simGetObject(int objID);
simVoid _simGetObjectLocalTransformation(const simVoid* object,simFloat* pos,simFloat* quat,simBool excludeFirstJointTransformation);
simVoid _simSetObjectLocalTransformation(simVoid* object,const simFloat* pos,const simFloat* quat);
simVoid _simSetObjectCumulativeTransformation(simVoid* object,const simFloat* pos,const simFloat* quat,simBool keepChildrenInPlace);
simVoid _simGetObjectCumulativeTransformation(const simVoid* object,simFloat* pos,simFloat* quat,simBool excludeFirstJointTransformation);
simBool _simIsShapeDynamicallyStatic(const simVoid* shape);
simInt _simGetTreeDynamicProperty(const simVoid* object);
simInt _simGetDummyLinkType(const simVoid* dummy,simInt* linkedDummyID);
simInt _simGetJointMode(const simVoid* joint);
simBool _simIsJointInHybridOperation(const simVoid* joint);
simVoid _simDisableDynamicTreeForManipulation(const simVoid* object,simBool disableFlag);
simBool _simIsShapeDynamicallyRespondable(const simVoid* shape);
simInt _simGetDynamicCollisionMask(const simVoid* shape);
const simVoid* _simGetLastParentForLocalGlobalCollidable(const simVoid* shape);
simVoid _simSetShapeIsStaticAndNotRespondableButDynamicTag(const simVoid* shape,simBool tag);
simBool _simGetShapeIsStaticAndNotRespondableButDynamicTag(const simVoid* shape);
simVoid _simSetJointPosition(const simVoid* joint,simFloat pos);
simFloat _simGetJointPosition(const simVoid* joint);
simVoid _simSetDynamicMotorPositionControlTargetPosition(const simVoid* joint,simFloat pos);
simVoid _simGetInitialDynamicVelocity(const simVoid* shape,simFloat* vel);
simVoid _simSetInitialDynamicVelocity(simVoid* shape,const simFloat* vel);
simVoid _simGetInitialDynamicAngVelocity(const simVoid* shape,simFloat* angularVel);
simVoid _simSetInitialDynamicAngVelocity(simVoid* shape,const simFloat* angularVel);
simBool _simGetStartSleeping(const simVoid* shape);
simBool _simGetWasPutToSleepOnce(const simVoid* shape);
simBool _simGetDynamicsFullRefreshFlag(const simVoid* object);
simVoid _simSetDynamicsFullRefreshFlag(const simVoid* object,simBool flag);
simVoid _simSetGeomProxyDynamicsFullRefreshFlag(simVoid* geomData,simBool flag);
simBool _simGetGeomProxyDynamicsFullRefreshFlag(const simVoid* geomData);
simBool _simGetParentFollowsDynamic(const simVoid* shape);
simVoid _simSetShapeDynamicVelocity(simVoid* shape,const simFloat* linear,const simFloat* angular);
simVoid _simGetAdditionalForceAndTorque(const simVoid* shape,simFloat* force,simFloat* torque);
simVoid _simClearAdditionalForceAndTorque(const simVoid* shape);
simBool _simGetJointPositionInterval(const simVoid* joint,simFloat* minValue,simFloat* rangeValue);
simInt _simGetJointType(const simVoid* joint);
simBool _simIsForceSensorBroken(const simVoid* forceSensor);
simVoid _simGetDynamicForceSensorLocalTransformationPart2(const simVoid* forceSensor,simFloat* pos,simFloat* quat);
simBool _simIsDynamicMotorEnabled(const simVoid* joint);
simBool _simIsDynamicMotorPositionCtrlEnabled(const simVoid* joint);
simBool _simIsDynamicMotorTorqueModulationEnabled(const simVoid* joint);
simVoid _simGetMotorPid(const simVoid* joint,simFloat* pParam,simFloat* iParam,simFloat* dParam);
simFloat _simGetDynamicMotorTargetPosition(const simVoid* joint);
simFloat _simGetDynamicMotorTargetVelocity(const simVoid* joint);
simFloat _simGetDynamicMotorMaxForce(const simVoid* joint);
simFloat _simGetDynamicMotorUpperLimitVelocity(const simVoid* joint);
simVoid _simSetDynamicMotorReflectedPositionFromDynamicEngine(simVoid* joint,simFloat pos);
simVoid _simSetJointSphericalTransformation(simVoid* joint,const simFloat* quat);
simVoid _simAddForceSensorCumulativeForcesAndTorques(simVoid* forceSensor,const simFloat* force,const simFloat* torque,int totalPassesCount);
simVoid _simAddJointCumulativeForcesOrTorques(simVoid* joint,simFloat forceOrTorque,int totalPassesCount);
simVoid _simSetDynamicJointLocalTransformationPart2(simVoid* joint,const simFloat* pos,const simFloat* quat);
simVoid _simSetDynamicForceSensorLocalTransformationPart2(simVoid* forceSensor,const simFloat* pos,const simFloat* quat);
simVoid _simSetDynamicJointLocalTransformationPart2IsValid(simVoid* joint,simBool valid);
simVoid _simSetDynamicForceSensorLocalTransformationPart2IsValid(simVoid* forceSensor,simBool valid);
const simVoid* _simGetGeomWrapFromGeomProxy(const simVoid* geomData);
simVoid _simGetLocalInertiaFrame(const simVoid* geomInfo,simFloat* pos,simFloat* quat);
simInt _simGetPurePrimitiveType(const simVoid* geomInfo);
simBool _simIsGeomWrapGeometric(const simVoid* geomInfo);
simBool _simIsGeomWrapConvex(const simVoid* geomInfo);
simInt _simGetGeometricCount(const simVoid* geomInfo);
simVoid _simGetAllGeometrics(const simVoid* geomInfo,simVoid** allGeometrics);
simVoid _simGetPurePrimitiveSizes(const simVoid* geometric,simFloat* sizes);
simVoid _simMakeDynamicAnnouncement(int announceType);
simVoid _simGetVerticesLocalFrame(const simVoid* geometric,simFloat* pos,simFloat* quat);
const simFloat* _simGetHeightfieldData(const simVoid* geometric,simInt* xCount,simInt* yCount,simFloat* minHeight,simFloat* maxHeight);
simVoid _simGetCumulativeMeshes(const simVoid* geomInfo,simFloat** vertices,simInt* verticesSize,simInt** indices,simInt* indicesSize);
simFloat _simGetMass(const simVoid* geomInfo);
simVoid _simGetPrincipalMomentOfInertia(const simVoid* geomInfo,simFloat* inertia);
simVoid _simGetGravity(simFloat* gravity);
simInt _simGetTimeDiffInMs(simInt previousTime);
simBool _simDoEntitiesCollide(simInt entity1ID,simInt entity2ID,simInt* cacheBuffer,simBool overrideCollidableFlagIfShape1,simBool overrideCollidableFlagIfShape2,simBool pathOrMotionPlanningRoutineCalling);
simBool _simGetDistanceBetweenEntitiesIfSmaller(simInt entity1ID,simInt entity2ID,simFloat* distance,simFloat* ray,simInt* cacheBuffer,simBool overrideMeasurableFlagIfNonCollection1,simBool overrideMeasurableFlagIfNonCollection2,simBool pathPlanningRoutineCalling);
simInt _simHandleJointControl(const simVoid* joint,simInt auxV,const simInt* inputValuesInt,const simFloat* inputValuesFloat,simFloat* outputValues);
simInt _simHandleCustomContact(simInt objHandle1,simInt objHandle2,simInt engine,simInt* dataInt,simFloat* dataFloat);
const simVoid* _simGetIkGroupObject(int ikGroupID);
simInt _simMpHandleIkGroupObject(const simVoid* ikGroup);
simFloat _simGetPureHollowScaling(const simVoid* geometric);
simInt _simGetJointCallbackCallOrder(const simVoid* joint);
simVoid _simDynCallback(const simInt* intData,const simFloat* floatData);


// Following courtesy of Stephen James:
simInt simExtLaunchUIThread(const simChar* applicationName,simInt options,const simChar* sceneOrModelOrUiToLoad,const simChar* applicationDir_);
simInt simExtCanInitSimThread();
simInt simExtSimThreadInit();
simInt simExtSimThreadDestroy();
simInt simExtPostExitRequest();
simInt simExtGetExitRequest();
simInt simExtStep(simBool stepIfRunning);
simInt simExtCallScriptFunction(simInt scriptHandleOrType, const simChar* functionNameAtScriptName,
                                               const simInt* inIntData, simInt inIntCnt,
                                               const simFloat* inFloatData, simInt inFloatCnt,
                                               const simChar** inStringData, simInt inStringCnt,
                                               const simChar* inBufferData, simInt inBufferCnt,
                                               simInt** outIntData, simInt* outIntCnt,
                                               simFloat** outFloatData, simInt* outFloatCnt,
                                               simChar*** outStringData, simInt* outStringCnt,
                                               simChar** outBufferData, simInt* outBufferSize);


// Deprecated begin
simInt simGetMaterialId(const simChar* materialName);
simInt simGetShapeMaterial(simInt shapeHandle);
simInt simHandleVarious();
simInt simSerialPortOpen(simInt portNumber,simInt baudRate,simVoid* reserved1,simVoid* reserved2);
simInt simSerialPortClose(simInt portNumber);
simInt simSerialPortSend(simInt portNumber,const simChar* data,simInt dataLength);
simInt simSerialPortRead(simInt portNumber,simChar* buffer,simInt dataLengthToRead);
simInt simJointGetForce(simInt jointHandle,simFloat* forceOrTorque);
simInt simGetPathPlanningHandle(const simChar* pathPlanningObjectName);
simInt simGetMotionPlanningHandle(const simChar* motionPlanningObjectName);
simInt simGetMpConfigForTipPose(simInt motionPlanningObjectHandle,simInt options,simFloat closeNodesDistance,simInt trialCount,const simFloat* tipPose,simInt maxTimeInMs,simFloat* outputJointPositions,const simFloat* referenceConfigs,simInt referenceConfigCount,const simFloat* jointWeights,const simInt* jointBehaviour,simInt correctionPasses);
simFloat* simFindMpPath(simInt motionPlanningObjectHandle,const simFloat* startConfig,const simFloat* goalConfig,simInt options,simFloat stepSize,simInt* outputConfigsCnt,simInt maxTimeInMs,simFloat* reserved,const simInt* auxIntParams,const simFloat* auxFloatParams);
simFloat* simSimplifyMpPath(simInt motionPlanningObjectHandle,const simFloat* pathBuffer,simInt configCnt,simInt options,simFloat stepSize,simInt increment,simInt* outputConfigsCnt,simInt maxTimeInMs,simFloat* reserved,const simInt* auxIntParams,const simFloat* auxFloatParams);
simFloat* simFindIkPath(simInt motionPlanningObjectHandle,const simFloat* startConfig,const simFloat* goalPose,simInt options,simFloat stepSize,simInt* outputConfigsCnt,simFloat* reserved,const simInt* auxIntParams,const simFloat* auxFloatParams);
simFloat* simGetMpConfigTransition(simInt motionPlanningObjectHandle,const simFloat* startConfig,const simFloat* goalConfig,simInt options,const simInt* select,simFloat calcStepSize,simFloat maxOutStepSize,simInt wayPointCnt,const simFloat* wayPoints,simInt* outputConfigsCnt,const simInt* auxIntParams,const simFloat* auxFloatParams);
simInt simCreateMotionPlanning(simInt jointCnt,const simInt* jointHandles,const simInt* jointRangeSubdivisions,const simFloat* jointMetricWeights,simInt options,const simInt* intParams,const simFloat* floatParams,const simVoid* reserved);
simInt simRemoveMotionPlanning(simInt motionPlanningHandle);
simInt simSearchPath(simInt pathPlanningObjectHandle,simFloat maximumSearchTime);
simInt simInitializePathSearch(simInt pathPlanningObjectHandle,simFloat maximumSearchTime,simFloat searchTimeStep);
simInt simPerformPathSearchStep(simInt temporaryPathSearchObject,simBool abortSearch);
simInt simLockInterface(simBool locked);
simInt simCopyPasteSelectedObjects();
simInt simResetPath(simInt pathHandle);
simInt simHandlePath(simInt pathHandle,simFloat deltaTime);
simInt simResetJoint(simInt jointHandle);
simInt simHandleJoint(simInt jointHandle,simFloat deltaTime);
simInt simAppendScriptArrayEntry(const simChar* reservedSetToNull,simInt scriptHandleOrType,const simChar* arrayNameAtScriptName,const simChar* keyName,const simChar* data,const simInt* what);
simInt simClearScriptVariable(const simChar* reservedSetToNull,simInt scriptHandleOrType,const simChar* variableNameAtScriptName);
simVoid _simGetJointOdeParameters(const simVoid* joint,simFloat* stopERP,simFloat* stopCFM,simFloat* bounce,simFloat* fudge,simFloat* normalCFM);
simVoid _simGetJointBulletParameters(const simVoid* joint,simFloat* stopERP,simFloat* stopCFM,simFloat* normalCFM);
simVoid _simGetOdeMaxContactFrictionCFMandERP(const simVoid* geomInfo,simInt* maxContacts,simFloat* friction,simFloat* cfm,simFloat* erp);
simBool _simGetBulletCollisionMargin(const simVoid* geomInfo,simFloat* margin,simInt* otherProp);
simBool _simGetBulletStickyContact(const simVoid* geomInfo);
simFloat _simGetBulletRestitution(const simVoid* geomInfo);
simVoid _simGetVortexParameters(const simVoid* object,simInt version,simFloat* floatParams,simInt* intParams);
simVoid _simGetNewtonParameters(const simVoid* object,simInt* version,simFloat* floatParams,simInt* intParams);
simVoid _simGetDamping(const simVoid* geomInfo,simFloat* linDamping,simFloat* angDamping);
simFloat _simGetFriction(const simVoid* geomInfo);
simInt simAddSceneCustomData(simInt header,const simChar* data,simInt dataLength);
simInt simGetSceneCustomDataLength(simInt header);
simInt simGetSceneCustomData(simInt header,simChar* data);
simInt simAddObjectCustomData(simInt objectHandle,simInt header,const simChar* data,simInt dataLength);
simInt simGetObjectCustomDataLength(simInt objectHandle,simInt header);
simInt simGetObjectCustomData(simInt objectHandle,simInt header,simChar* data);
simInt simCreateUI(const simChar* uiName,simInt menuAttributes,const simInt* clientSize,const simInt* cellSize,simInt* buttonHandles);
simInt simCreateUIButton(simInt uiHandle,const simInt* position,const simInt* size,simInt buttonProperty);
simInt simGetUIHandle(const simChar* uiName);
simInt simGetUIProperty(simInt uiHandle);
simInt simGetUIEventButton(simInt uiHandle,simInt* auxiliaryValues);
simInt simSetUIProperty(simInt uiHandle,simInt elementProperty);
simInt simGetUIButtonProperty(simInt uiHandle,simInt buttonHandle);
simInt simSetUIButtonProperty(simInt uiHandle,simInt buttonHandle,simInt buttonProperty);
simInt simGetUIButtonSize(simInt uiHandle,simInt buttonHandle,simInt* size);
simInt simSetUIButtonLabel(simInt uiHandle,simInt buttonHandle,const simChar* upStateLabel,const simChar* downStateLabel);
simChar* simGetUIButtonLabel(simInt uiHandle,simInt buttonHandle);
simInt simSetUISlider(simInt uiHandle,simInt buttonHandle,simInt position);
simInt simGetUISlider(simInt uiHandle,simInt buttonHandle);
simInt simSetUIButtonColor(simInt uiHandle,simInt buttonHandle,const simFloat* upStateColor,const simFloat* downStateColor,const simFloat* labelColor);
simInt simSetUIButtonTexture(simInt uiHandle,simInt buttonHandle,const simInt* size,const simChar* textureData);
simInt simCreateUIButtonArray(simInt uiHandle,simInt buttonHandle);
simInt simSetUIButtonArrayColor(simInt uiHandle,simInt buttonHandle,const simInt* position,const simFloat* color);
simInt simDeleteUIButtonArray(simInt uiHandle,simInt buttonHandle);
simInt simRemoveUI(simInt uiHandle);
simInt simSetUIPosition(simInt uiHandle,const simInt* position);
simInt simGetUIPosition(simInt uiHandle,simInt* position);
simInt simLoadUI(const simChar* filename,simInt maxCount,simInt* uiHandles);
simInt simSaveUI(simInt count,const simInt* uiHandles,const simChar* filename);
simInt simHandleGeneralCallbackScript(simInt callbackId,simInt callbackTag,simVoid* additionalData);
simInt simRegisterCustomLuaVariable(const simChar* varName,const simChar* varValue);
simInt simRegisterContactCallback(simInt(*callBack)(simInt,simInt,simInt,simInt*,simFloat*));
simInt simRegisterJointCtrlCallback(simInt(*callBack)(simInt,simInt,simInt,const simInt*,const simFloat*,simFloat*));
simInt simGetMechanismHandle(const simChar* mechanismName);
simInt simHandleMechanism(simInt mechanismHandle);
simInt simHandleCustomizationScripts(simInt callType);
simInt simSetVisionSensorFilter(simInt visionSensorHandle,simInt filterIndex,simInt options,const simInt* pSizes,const simUChar* bytes,const simInt* ints,const simFloat* floats,const simUChar* custom);
simInt simGetVisionSensorFilter(simInt visionSensorHandle,simInt filterIndex,simInt* options,simInt* pSizes,simUChar** bytes,simInt** ints,simFloat** floats,simUChar** custom);
simChar* simGetScriptSimulationParameter(simInt scriptHandle,const simChar* parameterName,simInt* parameterLength);
simInt simSetScriptSimulationParameter(simInt scriptHandle,const simChar* parameterName,const simChar* parameterValue,simInt parameterLength);
simInt simSetJointForce(simInt objectHandle,simFloat forceOrTorque);
// Deprecated end

""")

cwd = os.getcwd()
cffi_path = os.path.join(cwd, 'cffi_build')

ffibuilder.set_source(
    "pyrep.backend._sim_cffi",
    """
         #include "sim.h"   // the C header of the library
    """,
    libraries=['coppeliaSim'],
    library_dirs=[os.environ['COPPELIASIM_ROOT']],
    include_dirs=[cffi_path])

# For some reason, cffi makes it such that it looks for libv_rep.so.1
# rather than libv_rep.so. So we add a symlink.
path = os.path.join(os.environ['COPPELIASIM_ROOT'], 'libcoppeliaSim.so')
if not os.path.exists(path + '.1'):
    print('creating symlink: %s -> %s' % (path + '.1', path))
    os.symlink(path, path + '.1')

# Copy lua functions to the VREP_ROOT
print('copying lua file: %s -> %s' % ('pyrep/backend',
                                      os.environ['COPPELIASIM_ROOT']))
lua_script_fname = 'simAddOnScript_PyRep.lua'
copyfile(os.path.join('pyrep/backend', lua_script_fname),
         os.path.join(os.environ['COPPELIASIM_ROOT'], lua_script_fname))

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)
