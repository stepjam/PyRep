-- Additional PyRep functionality. To be placed in the CoppeliaSim root directory.

function sysCall_init()
end

function sysCall_cleanup()
end

function sysCall_addOnScriptSuspend()
end

function sysCall_addOnScriptResume()
end

function sysCall_nonSimulation()
end

function sysCall_beforeMainScript()
end

function sysCall_beforeInstanceSwitch()
end

function sysCall_afterInstanceSwitch()
end

function sysCall_beforeSimulation()
end

function sysCall_afterSimulation()
end

_getConfig=function(jh)
    -- Returns the current robot configuration
    local config={}
    for i=1,#jh,1 do
        config[i]=sim.getJointPosition(jh[i])
    end
    return config
end

_setConfig=function(jh, config)
    -- Applies the specified configuration to the robot
    if config then
        for i=1,#jh,1 do
            sim.setJointPosition(jh[i],config[i])
        end
    end
end

_getConfigDistance=function(jointHandles,config1,config2)
    -- Returns the distance (in configuration space) between two configurations
    local d=0
    for i=1,#jointHandles,1 do
        -- TODO *metric[i] should be here to give a weight to each joint.
        local dx=(config1[i]-config2[i])*1.0
        d=d+dx*dx
    end
    return math.sqrt(d)
end

_sliceFromOffset=function(array, offset)
    sliced = {}
    for i=1,#array-offset,1 do
        sliced[i] = array[i+offset]
    end
    return sliced
end

_findPath=function(goalConfigs,cnt,jointHandles,algorithm,collisionPairs)
    -- Here we do path planning between the specified start and goal configurations. We run the search cnt times,
    -- and return the shortest path, and its length

    local startConfig = _getConfig(jointHandles)
    local task=simOMPL.createTask('task')
    simOMPL.setVerboseLevel(task, 0)

    alg = _getAlgorithm(algorithm)

    simOMPL.setAlgorithm(task,alg)

    local jSpaces={}
    for i=1,#jointHandles,1 do
        jh = jointHandles[i]
        cyclic, interval = sim.getJointInterval(jh)
        -- If there are huge intervals, then limit them
        if interval[1] < -6.28 and interval[2] > 6.28 then
            pos=sim.getJointPosition(jh)
            interval[1] = -6.28
            interval[2] = 6.28
        end
        local proj=i
        if i>3 then proj=0 end
        jSpaces[i]=simOMPL.createStateSpace('j_space'..i,simOMPL.StateSpaceType.joint_position,jh,{interval[1]},{interval[2]},proj)
    end

    simOMPL.setStateSpace(task, jSpaces)
    if collisionPairs ~= nil then
        simOMPL.setCollisionPairs(task, collisionPairs)
    end
    simOMPL.setStartState(task, startConfig)
    simOMPL.setGoalState(task, goalConfigs[1])
    for i=2,#goalConfigs,1 do
        simOMPL.addGoalState(task,goalConfigs[i])
    end
    local path=nil
    local l=999999999999
    for i=1,cnt,1 do
        search_time = 4
        local res,_path=simOMPL.compute(task,search_time,-1,300)

        -- Path can sometimes touch on invalid state during simplifying
        if res and _path then
            local is_valid=true
            local jhl=#jointHandles
            local pc=#_path/jhl
            for i=1,pc-1,1 do
                local config={}
                for j=1,jhl,1 do
                    config[j]=_path[(i-1)*jhl+j]
                end
                is_valid=simOMPL.isStateValid(task, config)
                if not is_valid then
                    break
                end
            end

            if is_valid then
                local _l=_getPathLength(_path, jointHandles)
                if _l<l then
                    l=_l
                    path=_path
                end
            end
        end
    end
    simOMPL.destroyTask(task)
    return path,l
end

_getAlgorithm=function(algorithm)
    -- Returns correct algorithm functions from user string
    alg = nil
    if algorithm == 'BiTRRT' then
        alg = simOMPL.Algorithm.BiTRRT
    elseif algorithm == 'BITstar' then
        alg = simOMPL.Algorithm.BITstar
    elseif algorithm == 'BKPIECE1' then
        alg = simOMPL.Algorithm.BKPIECE1
    elseif algorithm == 'CForest' then
        alg = simOMPL.Algorithm.CForest
    elseif algorithm == 'EST' then
        alg = simOMPL.Algorithm.EST
    elseif algorithm == 'FMT' then
        alg = simOMPL.Algorithm.FMT
    elseif algorithm == 'KPIECE1' then
        alg = simOMPL.Algorithm.KPIECE1
    elseif algorithm == 'LazyPRM' then
        alg = simOMPL.Algorithm.LazyPRM
    elseif algorithm == 'LazyPRMstar' then
        alg = simOMPL.Algorithm.LazyPRMstar
    elseif algorithm == 'LazyRRT' then
        alg = simOMPL.Algorithm.LazyRRT
    elseif algorithm == 'LBKPIECE1' then
        alg = simOMPL.Algorithm.LBKPIECE1
    elseif algorithm == 'LBTRRT' then
        alg = simOMPL.Algorithm.LBTRRT
    elseif algorithm == 'PDST' then
        alg = simOMPL.Algorithm.PDST
    elseif algorithm == 'PRM' then
        alg = simOMPL.Algorithm.PRM
    elseif algorithm == 'PRMstar' then
        alg = simOMPL.Algorithm.PRMstar
    elseif algorithm == 'pRRT' then
        alg = simOMPL.Algorithm.pRRT
    elseif algorithm == 'pSBL' then
        alg = simOMPL.Algorithm.pSBL
    elseif algorithm == 'RRT' then
        alg = simOMPL.Algorithm.RRT
    elseif algorithm == 'RRTConnect' then
        alg = simOMPL.Algorithm.RRTConnect
    elseif algorithm == 'RRTstar' then
        alg = simOMPL.Algorithm.RRTstar
    elseif algorithm == 'SBL' then
        alg = simOMPL.Algorithm.SBL
    elseif algorithm == 'SPARS' then
        alg = simOMPL.Algorithm.SPARS
    elseif algorithm == 'SPARStwo' then
        alg = simOMPL.Algorithm.SPARStwo
    elseif algorithm == 'STRIDE' then
        alg = simOMPL.Algorithm.STRIDE
    elseif algorithm == 'TRRT' then
        alg = simOMPL.Algorithm.TRRT
    end
    return alg
end

_getPathLength=function(path, jointHandles)
    -- Returns the length of the path in configuration space
    local d=0
    local l=#jointHandles
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1, config2 = _beforeAfterConfigFromPath(path, i, l)
        d=d+_getConfigDistance(jointHandles,config1,config2)
    end
    return d
end

_beforeAfterConfigFromPath=function(path, path_index, num_handles)
    local config1 = {}
    local config2 = {}
    for i=1,num_handles,1 do
        config1[i] = path[(path_index-1)*num_handles+i]
        config2[i] = path[path_index*num_handles+i]
    end
    return config1, config2
end

_getPoseOnPath=function(pathHandle, relativeDistance)
    local pos = sim.getPositionOnPath(pathHandle, relativeDistance)
    local ori = sim.getOrientationOnPath(pathHandle, relativeDistance)
    return pos, ori
end

getNonlinearPath=function(inInts,inFloats,inStrings,inBuffer)
    algorithm = inStrings[1]
    collisionHandle = inInts[1]
    ignoreCollisions = inInts[2]
    searchCntPerGoalConfig = inInts[3]
    jointHandles = _sliceFromOffset(inInts, 3)
    collisionPairs={collisionHandle, sim.handle_all}
    if ignoreCollisions==1 then
        collisionPairs=nil
    end

    local configCnt = #inFloats/#jointHandles
    goalConfigs = {}
    for i=1,configCnt,1 do
        local config={}
        for j=1,#jointHandles,1 do
            table.insert(config, inFloats[((i-1) * #jointHandles)+j])
        end
        table.insert(goalConfigs, config)
    end

    -- Search a path from current config to a goal config.
    path = _findPath(goalConfigs, searchCntPerGoalConfig, jointHandles, algorithm, collisionPairs)
    if path == nil then
        path = {}
    end
    return {},path,{},''
end

getPathFromCartesianPath=function(inInts,inFloats,inStrings,inBuffer)
    pathHandle = inInts[1]
    ikGroup = inInts[2]
    ikTarget = inInts[3]
    jointHandles = _sliceFromOffset(inInts, 3)
    collisionPairs = nil--{collisionHandle, sim.handle_all}
    orientationCorrection = inFloats

    local initIkPos = sim.getObjectPosition(ikTarget, -1)
    local initIkOri = sim.getObjectOrientation(ikTarget, -1)
    local originalConfig = _getConfig(jointHandles)
    local i = 0.05
    local fullPath = {}
    local failed = false

    while i <= 1.0 do
        pos, ori = _getPoseOnPath(pathHandle, i)
        sim.setObjectPosition(ikTarget, -1, pos)
        sim.setObjectOrientation(ikTarget, -1, ori)
        intermediatePath = sim.generateIkPath(ikGroup,jointHandles,20,collisionPairs)
        if intermediatePath == nil then
            failed = true
            break
        end
        for j=1,#intermediatePath,1 do
            table.insert(fullPath, intermediatePath[j])
        end
        newConfig = {}
        for j=#intermediatePath-#jointHandles+1,#intermediatePath,1 do
            table.insert(newConfig, intermediatePath[j])
        end
        _setConfig(jointHandles, newConfig)
        i = i + 0.05
    end
    _setConfig(jointHandles, originalConfig)
    sim.setObjectPosition(ikTarget, -1, initIkPos)
    sim.setObjectOrientation(ikTarget, -1, initIkOri)
    if failed then
        fullPath = {}
    end
    return {},fullPath,{},''
end

insertPathControlPoint=function(inInts,inFloats,inStrings,inBuffer)
    local handle = inInts[1]
    local ptCnt = inInts[2]
    local floatSkip = 6
    local ptData = {}
    for i=1,ptCnt,1 do
        local offset = (i-1)*floatSkip
        local ctrPos = {inFloats[offset+1], inFloats[offset+2], inFloats[offset+3]}
        local ctrOri = {inFloats[offset+4], inFloats[offset+5], inFloats[offset+6]}
        local vel = 0
        local virDist = 0
        local bezierPointsAtControl = 20
        local bazierInterpolFactor1 = 0.990
        local bazierInterpolFactor2 = 0.990
        local auxFlags = 0
        table.insert(ptData, ctrPos[1])
        table.insert(ptData, ctrPos[2])
        table.insert(ptData, ctrPos[3])
        table.insert(ptData, ctrOri[1])
        table.insert(ptData, ctrOri[2])
        table.insert(ptData, ctrOri[3])
        table.insert(ptData, vel)
        table.insert(ptData, virDist)
        table.insert(ptData, bezierPointsAtControl)
        table.insert(ptData, bazierInterpolFactor1)
        table.insert(ptData, bazierInterpolFactor2)
    end
    res = sim.insertPathCtrlPoints(handle, 0, 0, ptCnt, ptData)
    return {},{},{},''
end

getBoxAdjustedMatrixAndFacingAngle=function(inInts,inFloats,inStrings,inBuffer)
    local baseHandle = inInts[1]
    local targetHandle = inInts[2]
    local p2=sim.getObjectPosition(targetHandle,-1)
    local p1=sim.getObjectPosition(baseHandle,-1)
    local p={p2[1]-p1[1],p2[2]-p1[2],p2[3]-p1[3]}
    local pl=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
    p[1]=p[1]/pl
    p[2]=p[2]/pl
    p[3]=p[3]/pl
    local m=sim.getObjectMatrix(targetHandle,-1)
    local matchingScore=0
    for i=1,3,1 do
        v={m[0+i],m[4+i],m[8+i]}
        score=v[1]*p[1]+v[2]*p[2]+v[3]*p[3]
        if (math.abs(score)>matchingScore) then
            s=1
            if (score<0) then s=-1 end
            matchingScore=math.abs(score)
            bestMatch={v[1]*s,v[2]*s,v[3]*s}
        end
    end
    angle=math.atan2(bestMatch[2],bestMatch[1])
    m=sim.buildMatrix(p2,{0,0,angle})

    table.insert(m,angle-math.pi/2)

    return {},m,{},''
end

getNonlinearPathMobile=function(inInts,inFloats,inStrings,inBuffer)
    algorithm = inStrings[1]
    robotHandle = inInts[1]
    targetHandle = inInts[2]
    collisionHandle=inInts[3]
    ignoreCollisions=inInts[4]
    bd=inFloats[1]
    path_pts=inInts[5]

    collisionPairs={collisionHandle,sim.handle_all}

    if ignoreCollisions==1 then
        collisionPairs=nil
    end

    t=simOMPL.createTask('t')
    simOMPL.setVerboseLevel(t, 0)
    ss=simOMPL.createStateSpace('2d',simOMPL.StateSpaceType.dubins,robotHandle,{-bd,-bd},{bd,bd},1)
    state_h = simOMPL.setStateSpace(t,{ss})
    simOMPL.setDubinsParams(ss,0.1,true)
    simOMPL.setAlgorithm(t,_getAlgorithm(algorithm))

    if collisionPairs ~= nil then
        simOMPL.setCollisionPairs(t, collisionPairs)
    end

    startpos=sim.getObjectPosition(robotHandle,-1)
    startorient=sim.getObjectOrientation(robotHandle,-1)
    startpose={startpos[1],startpos[2],startorient[3]}

    simOMPL.setStartState(t,startpose)

    goalpos=sim.getObjectPosition(targetHandle,-1)
    goalorient=sim.getObjectOrientation(targetHandle,-1)
    goalpose={goalpos[1],goalpos[2],goalorient[3]}

    simOMPL.setGoalState(t,goalpose)

    r,path=simOMPL.compute(t,4,-1,path_pts)

    simOMPL.destroyTask(t)

    return {},path,{},''
end

handleSpherical=function(inInts,inFloats,inStrings,inBuffer)
    local depth_handle=inInts[1]
    local rgb_handle=inInts[2]
    local six_sensor_handles = {inInts[3], inInts[4], inInts[5], inInts[6], inInts[7], inInts[8]}
    simVision.handleSpherical(rgb_handle, six_sensor_handles, 360, 180, depth_handle)
    return {},{},{},''
end