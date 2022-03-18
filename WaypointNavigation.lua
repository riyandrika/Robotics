-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end

function createRandomBumpyFloor()
    print ("Generating new random bumpy floor.")
    sim.setThreadAutomaticSwitch(false)

    -- Remove existing bumpy floor if there already is one
    if (heightField ~= nil) then
        sim.setObjectPosition(heightField, heightField, {0.05, 0, 0})
        return
    end
    --  Create random bumpy floor for robot to drive on
    floorSize = 5
    --heightFieldResolution = 0.3
    --heightFieldNoise = 0.00000005
    heightFieldResolution = 0.1
    heightFieldNoise = 0.0000008
    cellsPerSide = floorSize / heightFieldResolution
    cellHeights = {}
    for i=1,cellsPerSide*cellsPerSide,1 do
        table.insert(cellHeights, gaussian(0, heightFieldNoise))
    end
    heightField=sim.createHeightfieldShape(0, 0, cellsPerSide, cellsPerSide, floorSize, cellHeights)
    -- Make the floor invisible
    sim.setObjectInt32Param(heightField,10,0)
    sim.setThreadAutomaticSwitch(true)
end


function get_walls()
    -- Disable error reporting
    local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
    sim.setInt32Param(sim.intparam_error_report_mode,0)
    local N = 1
    while true do
        local handle = sim.getObjectHandle("Wall"..tostring(N))
        if handle <= 0 then
            break
        end

        -- Read position and shape of wall
        -- Assume here that it is thin and oriented either along the x axis or y axis

        -- We can now get the propertries of these walls, e.g....
        local pos = sim.getObjectPosition(handle, -1)
        local res,minx = sim.getObjectFloatParameter(handle,15)
        local res,maxx = sim.getObjectFloatParameter(handle,18)
        local res,miny = sim.getObjectFloatParameter(handle,16)
        local res,maxy = sim.getObjectFloatParameter(handle,19)
    
        --print("Position of Wall " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))
        --print("minmax", minx, maxx, miny, maxy)
 
        local Ax, Ay, Bx, By
        if (maxx - minx > maxy - miny) then
            print("Wall " ..tostring(N).. " along x axis")
            Ax = pos[1] + minx
            Ay = pos[2]
            Bx = pos[1] + maxx
            By = pos[2]
        else
            print("Wall " ..tostring(N).. " along y axis")
            Ax = pos[1]
            Ay = pos[2] + miny
            Bx = pos[1]
            By = pos[2] + maxy
        end
        print (Ax, Ay, Bx, By)

        walls[N] = {Ax, Ay, Bx, By}
        N = N + 1
    end
    -- enable error reporting
    sim.setInt32Param(sim.intparam_error_report_mode,savedState)

    return N - 1
end

function distance_to_wall(xPose,yPose,thetaPose)
    
    -- Declare distance from wall array
    m = {}
    
    -- For every wall
    for i=1, N_WALLS do
        -- Obtain coordinates of wall
        Ax = walls[i][1]
        Ay = walls[i][2]
        Bx = walls[i][3]
        By = walls[i][4]
        
        -- Obtain robot's distance from each wall
        m[i] = (((By-Ay)*(Ax-xPose))-((Bx-Ax)*(Ay-yPose)))/(((By-Ay)*math.cos(thetaPose))-((Bx-Ax)*math.sin(thetaPose)))
        
        -- reject negative distances
        if (m[i] < 0)  then
            m[i] = math.huge
        end
        
        -- check if intersect is at valid point (along the wall)
        x_intersection = xPose + m[i]*math.cos(thetaPose)
        y_intersection = yPose + m[i]*math.sin(thetaPose)
        
        if (x_intersection < math.min(Ax,Bx) or x_intersection > math.max(Ax,Bx) or y_intersection < math.min(Ay,By) or y_intersection > math.max(Ay,By)) then
            m[i] = math.huge
        end
    end    
    
    key = 0
    min_m = math.huge
    
    for k, v in ipairs(m) do
        if m[k] < min_m then
            key, min_m = k, v
        end
    end
    return(min_m)
    
end

function calculateLikelihood(z)
    for i =1, N do
        m,wall = distance_to_wall(xArray[i],yArray[i],thetaArray[i])
        diff = z - m
        likelihood = math.exp((-(diff^2))/(2*sensorVariance))
        weightArray[i] = weightArray[i]*likelihood
    end
end

function normalisation()
    local sum = 0
    for i =1, #weightArray do
        sum = sum + weightArray[i]
    end
    for i =1, #weightArray do
        weightArray[i] = weightArray[i]/sum
    end
    --print("normalisation",weightArray)
end

function resampling()
    
    -- create cumulative weight array
    for i =2, #weightArray do
        weightArray[i] = weightArray[i-1] + weightArray[i]
    end
    --print("cumulative weight array",weightArray)
    
    particle = 0 -- index of particle to be sampled
    for i = 1, N do
        -- generate random number
        selector = math.random()
        
        -- iterate through cumulative array to find range selector is in
        for j = 1,#weightArray do
            if (selector<weightArray[j]) then
                particle = j
                break
            end
        end
        
        -- store the sampled coordinates into new arrays (xArrayNew, yArrayNew, thetaArrayNew)
        xArrayNew[i] = xArray[particle]
        yArrayNew[i] = yArray[particle]
        thetaArrayNew[i] = thetaArray[particle]
    end
    
    -- can i equate array to new array? YES!
    for i = 1, N do
        xArray[i] = xArrayNew[i]
        yArray[i] = yArrayNew[i]
        thetaArray[i] = thetaArrayNew[i]
    end
    
end

function estimate_current_pose(xArray, yArray, thetaArray)
    local pose_array = {}
    local sum = 0
    for i = 1, #xArray do
        sum = sum + xArray[i]
    end
    pose_array[1] = sum/#xArray
    
    sum = 0
    for i = 1, #yArray do
        sum = sum + yArray[i]
    end
    pose_array[2] = sum/#yArray
    
            sum = 0
    for i = 1, #thetaArray do
        sum = sum + thetaArray[i]
    end
    pose_array[3] = sum/#thetaArray
    return pose_array
end

function getMaxMotorAngleFromTarget(posL, posR)

    -- How far are the left and right motors from their targets? Find the maximum
    maxAngle = 0
    if (speedBaseL > 0) then
        remaining = motorAngleTargetL - posL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseL < 0) then
        remaining = posL - motorAngleTargetL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR > 0) then
        remaining = motorAngleTargetR - posR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR < 0) then
        remaining = posR - motorAngleTargetR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end

    return maxAngle
end

function recalibrate()
    calculateLikelihood(noisyDistance)
    normalisation()
    resampling()
    
    -- reset weights and set new dummy positions
    for i=1, N do
        weightArray[i] = 1/N
        sim.setObjectPosition(dummyArray[i], -1, {xArray[i], yArray[i], 0}) -- Cartesian space
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]}) -- roll/pitch/yaw
    end
    
    -- estimate new parameters
    estimate_array = estimate_current_pose(xArray,yArray,thetaArray)
    x_estimate = estimate_array[1]                      
    y_estimate = estimate_array[2]                      
    theta_estimate = estimate_array[3]
    --find distance D
    D = math.sqrt((x_waypoint - x_estimate)^2 + (y_waypoint - y_estimate)^2)
    --find turn_angle
    target_angle = math.atan2(y_waypoint - y_estimate,x_waypoint - x_estimate)
    turn_angle = target_angle - theta_estimate
    turn_angle = turn_angle%(2*math.pi)
    if (turn_angle > math.pi) then
        turn_angle = turn_angle-(2*math.pi)
    end
    
    print("waypoint",x_waypoint,y_waypoint)
    print("D",D,"turn angle",math.deg(turn_angle))
    print("theta_estimate",math.deg(theta_estimate))
    print("target angle",math.deg(target_angle))
    
end

-- This function is executed exactly once when the scene is initialised
function sysCall_init()

    tt = sim.getSimulationTime()
    print("Init hello", tt)
          
    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")
 
    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

   
    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5
    speedBaseL = 0
    speedBaseR = 0
    turn_angle = 0
    D = 0
    
    waypoint_counter = 0

    -- Which step are we in?
    -- 0 is a dummy value which is immediately completed
    stepCounter = 0
    stepCompletedFlag = false
    stepList = {}
    
    x = 0
    y = 0
        
    stepList[1] = {"set_waypoint"}
    stepList[2] = {"turn"}
    stepList[3] = {"stop"}
    stepList[4] = {"forward"}
    stepList[5] = {"repeat"}
    
    N_WAYPOINTS = 26
    currentWaypoint = 0
    waypoints = {}
    waypoints[1] = {0.5,0}
    waypoints[2] = {1,0}
    waypoints[3] = {1,0.5}
    waypoints[4] = {1,1}
    waypoints[5] = {1,1.5}
    waypoints[6] = {1,2}
    waypoints[7] = {0.5,2}
    waypoints[8] = {0,2}
    waypoints[9] = {-0.5,2}
    waypoints[10] = {-1,2}
    waypoints[11] = {-1,1.5}
    waypoints[12] = {-1,1}
    waypoints[13] = {-1.5,1}
    waypoints[14] = {-2,1}
    waypoints[15] = {-2,0.5}
    waypoints[16] = {-2,0}
    waypoints[17] = {-2,-0.5}
    waypoints[18] = {-1.5,-1}
    waypoints[19] = {-1,-1.5}
    waypoints[20] = {-0.5,-1.5}
    waypoints[21] = {0,-1.5}
    waypoints[22] = {0.5,-1.5}
    waypoints[23] = {1,-1.5} 
    waypoints[24] = {1,-1}
    waypoints[25] = {0.5,-0.5}
    waypoints[26] = {0,0}


 
 
    -- Data structure for walls
    walls = {}
    -- Fill it by parsing the scene in the GUI
    N_WALLS = get_walls()
    -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates
  
    sensorStandardDeviation = 0.1
    sensorVariance = sensorStandardDeviation^2
    noisyDistance = 0
 
    -- Create and initialise arrays for particles, and display them with dummies
    xArray = {}
    yArray = {}
    thetaArray = {}
    xArrayNew = {}
    yArrayNew = {}
    thetaArrayNew = {}
    weightArray = {}
    dummyArray = {}
    
    N = 100
    for i=1, N do
        xArray[i] = 0
        yArray[i] = 0
        thetaArray[i] = 0
        xArrayNew[i] = 0
        yArrayNew[i] = 0
        thetaArrayNew[i] = 0
        weightArray[i] = 1/N
        dummyArray[i] = sim.createDummy(0.05)
        sim.setObjectPosition(dummyArray[i], -1, {0,0,0})
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,0})
    end

    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

     -- To calibrate
    motorAnglePerMetre = 24.8
    motorAnglePerRadian = 3.05
 

end

function sysCall_actuation()
    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)
    
    result,cleanDistance=sim.readProximitySensor(turretSensor)

    if (result>0) then
        noisyDistance= cleanDistance + gaussian(0.0, sensorVariance)
        -- print ("Depth sensor reading ", noisyDistance)
        -- print(calculateLikelihood(0,0,0,noisyDistance))

    end

    -- Get and plot current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)
    


    -- Start new step?
    if (stepCompletedFlag == true or stepCounter == 0) then
        stepCounter = stepCounter + 1
        stepCompletedFlag = false


        newStepType = stepList[stepCounter][1]

        if (newStepType == "repeat") then
            -- Loop back to the first step
            stepCounter = 1
            newStepType = stepList[stepCounter][1]
        end

        print("New step:", stepCounter, newStepType)
 
        if (newStepType == "forward") then
            -- Forward step: set new joint targets
            recalibrate()
            newStepAmount = D
            motorAngleTargetL = posL + newStepAmount * motorAnglePerMetre
            motorAngleTargetR = posR + newStepAmount * motorAnglePerMetre
            
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            recalibrate()
            newStepAmount = turn_angle
            motorAngleTargetL = posL - newStepAmount * motorAnglePerRadian
            motorAngleTargetR = posR + newStepAmount * motorAnglePerRadian
            
        elseif (newStepType == "stop") then
            print ("Stopping!")
        end
    end


    -- Handle current ongoing step
    stepType = stepList[stepCounter][1]

    if (stepType == "turn") then
        if (turn_angle >= 0) then
            speedBaseL = -speedBase
            speedBaseR = speedBase
        else
            speedBaseL = speedBase
            speedBaseR = -speedBase
        end
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true
            --[[calculateLikelihood(noisyDistance) -- update weightArray with likelihood
            normalisation() 
            resampling() -- gives new positions for particles--]]
            for i=1, N do
                g = gaussian(0,math.rad(0.1))
                thetaArray[i] = thetaArray[i] + turn_angle + g
                --weightArray[i] = 1/N
                sim.setObjectPosition(dummyArray[i], -1, {xArray[i], yArray[i], 0}) -- Cartesian space
                sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]}) -- roll/pitch/yaw
            end
        end
        
    elseif (stepType == "forward") then
        speedBaseL = speedBase
        speedBaseR = speedBase
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true
            --[[calculateLikelihood(noisyDistance)
            normalisation()
            resampling()--]]
            for i=1, N do
                e = gaussian(0,D*.1^2)
                f = gaussian(0,math.rad(0.05))
                xArray[i] = xArray[i] + (D + e)*math.cos(thetaArray[i])
                yArray[i] = yArray[i] + (D + e)*math.sin(thetaArray[i])
                thetaArray[i] = thetaArray[i] + f
                --weightArray[i] = 1/N
                sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
                sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
            end
        end
        
    elseif (stepType == "stop") then
        speedBaseL = 0
        speedBaseR = 0

-- Check to see if the robot is stationary to within a small threshold
        linearVelocity,angularVelocity=sim.getVelocity(robotBase)
        vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
        --print ("stop", linearVelocity, vLin, vAng)
    
        if (vLin < 0.001 and vAng < 0.01) then
            stepCompletedFlag = true
        end
    elseif (stepType == "set_waypoint") then
        waypoint_counter = waypoint_counter + 1
        x_waypoint = waypoints[waypoint_counter][1]
        y_waypoint = waypoints[waypoint_counter][2]
        
        stepCompletedFlag = true
        
        
        --[[calculateLikelihood(noisyDistance) -- update weightArray with likelihood
        normalisation() 
        resampling() -- gives new positions for particles
        
        estimate_array = estimate_current_pose(xArray,yArray,thetaArray)
        x_estimate = estimate_array[1]                      
        y_estimate = estimate_array[2]                      
        theta_estimate = estimate_array[3]
        --find distance D
        D = math.sqrt((x_waypoint - x_estimate)^2 + (y_waypoint - y_estimate)^2)
        --find turn_angle
        target_angle = math.atan2(y_waypoint - y_estimate,x_waypoint - x_estimate)
        turn_angle = target_angle - theta_estimate
        turn_angle = turn_angle%(2*math.pi)
        if (turn_angle > math.pi) then
            turn_angle = turn_angle-(2*math.pi)
        end
        stepCompletedFlag = true
        print("waypoint",x_waypoint,y_waypoint)
        print("D",D,"turn angle",math.deg(turn_angle))
        print("theta_estimate",math.deg(theta_estimate))
        print("target angle",math.deg(target_angle))--]]
        
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)        

    
end

function sysCall_cleanup()
    --simUI.destroy(ui)
end 
