-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

local FabrikSolver = Object.new("FabrikSolver")

function FabrikSolver.new(LimbVectorTable, IteratedLimbVectorTable, LimbLengthTable, Motor6DTable,LimbChain)
    local obj = FabrikSolver:make()

    obj.LimbChain = LimbChain
    -- Initialize the object class
    obj.LimbVectorTable = LimbVectorTable
    obj.IteratedLimbVectorTable = IteratedLimbVectorTable
    obj.LimbLengthTable = LimbLengthTable
    obj.LimbConstraintTable = nil

    obj.Motor6DTable = Motor6DTable
    if Motor6DTable then
        obj.FirstJointC0 = Motor6DTable[1].C0

        local LimbCFrameTable = {}

        for i, Motor6D in pairs(Motor6DTable) do
            LimbCFrameTable[i] = Motor6D.Part0.CFrame
        end 

        obj.LimbCFrameTable = LimbCFrameTable
    end
    
    -- Initialize number for summing
    local MaxLength = 0
    -- adds all the magnitudes of the limb vector
    for i = 1, #LimbLengthTable, 1 do
        MaxLength = MaxLength + LimbLengthTable[i]
    end

    obj.MaxLength = MaxLength

    return obj
end

--[[---------------------------------------------------------
    Executes one iteration of the Fabrik algo or not depending on the tolerance level
]]
function FabrikSolver:IterateOnce(originCF, targetPosition, tolerance)

    if self.Motor6DTable then
        self:RotateLimb()
    end

    -- initialize measure feet to where it should be in the world position
    local vectorSum = Vector3.new(0, 0, 0)
    for i = 1, #self.IteratedLimbVectorTable, 1 do
        vectorSum = vectorSum + self.IteratedLimbVectorTable[i]
    end
    local feetJoint = originCF.Position + vectorSum
    local feetToTarget = targetPosition - feetJoint
    local distanceToGoal = feetToTarget.Magnitude

    --[[ Measures origin joint to target position, not needed for now due to how constraints work
    local originToGoalLength = (targetPosition-originCF.Position).Magnitude
    if originToGoalLength<self.MaxLength then
        print("in range")
    else
        print("out of range")
    end
    ]]

    -- target point is "reachable"
    -- if Distance is more than tolerance then iterate to move the new vectors closer
    -- If not then don't execute the iteration to save FPS

    if distanceToGoal >= tolerance then

        -- Do backwards on itself first then forwards
        self:Backwards(originCF, targetPosition)
        self:Forwards(originCF, targetPosition)

        return self.IteratedLimbVectorTable

    else
        -- Limb is within tolerance/already reached goal so don't do anything

        return self.IteratedLimbVectorTable
    end

end

function FabrikSolver:IterateUntilGoal(originCF, targetPosition, tolerance, InputtedMaxBreakCount)

    -- initialize measure feet to where it should be in the world position
    local vectorSum = Vector3.new(0, 0, 0)
    for i = 1, #self.IteratedLimbVectorTable, 1 do
        vectorSum = vectorSum + self.IteratedLimbVectorTable[i]
    end
    local feetJoint = originCF.Position + vectorSum
    local feetToTarget = targetPosition - feetJoint
    local distanceToGoal = feetToTarget.Magnitude

    -- target point is "reachable"
    -- if Distance is more than tolerance then iterate to move the new vectors closer
    -- If not then don't execute the iteration to save FPS
    -- Now includes a maximum iteration count
    local maxBreakCount
    if InputtedMaxBreakCount and type(InputtedMaxBreakCount) == "number" then
        maxBreakCount = InputtedMaxBreakCount
    else
        -- default is maximum 10 iterations
        maxBreakCount = 10
    end

    --print("maxbreak count: ",maxBreakCount)
    local bcount = 0

    while distanceToGoal >= tolerance do

        -- Do backwards on itself first then forwards until it reaches goal
        self:Backwards(originCF, targetPosition)
        self:Forwards(originCF, targetPosition)
        
        --Issue constraints don't update unless motors are updated
        --Solution update the motors lol
        self.LimbChain:UpdateMotors()

        --measure distance again
        -- initialize measure feet to where it should be in the world position
        local vectorSum = Vector3.new(0, 0, 0)
        for i = 1, #self.IteratedLimbVectorTable, 1 do
          vectorSum = vectorSum + self.IteratedLimbVectorTable[i]
        end
        local feetJoint = originCF.Position + vectorSum
        local feetToTarget = targetPosition - feetJoint
        distanceToGoal = feetToTarget.Magnitude
        
        --Counts the amount of iterations, if impossible to solve stops after max default at 10 iterations
        bcount += 1
        if bcount > maxBreakCount then 
            --print("bcount:", bcount,"failed to reach goal: ", distanceToGoal)
            return self.IteratedLimbVectorTable 
        end

    end
    --Iterate once in case
    self:Backwards(originCF, targetPosition)
    self:Forwards(originCF, targetPosition)

    -- Limb is within tolerance/already reached goal so don't do anything
    --print("bcount:", bcount,"Reached goal: ", distanceToGoal)
    return self.IteratedLimbVectorTable

end

--[[
    Constraining the forwards operation only makes it weird I dont think it simply does enough
    So constraints are also applied to backwards now
    Now is generalized with less parameters thanks to meta table objects storing the properties
]]
function FabrikSolver:Backwards(originCF, targetPos)

    -- Transporting from module scrip to class so gotta do this
    local IteratedLimbVectorTable = self.IteratedLimbVectorTable
    local limbLengthTable = self.LimbLengthTable
    local limbConstraintTable = self.LimbConstraintTable

    local vectorSumFromOrigin = Vector3.new()
    -- Iterate through all the limb vectors and performs the backwards operation
    for i = #IteratedLimbVectorTable, 1, -1 do
        local vectorSum = Vector3.new(0, 0, 0)

        for v = 1, i - 1, 1 do vectorSum = vectorSum + IteratedLimbVectorTable[v] end

        local pointTowards = originCF.Position + vectorSum
        local pointFrom = targetPos + vectorSumFromOrigin

        -- Gets the new direction of the new vector along the chain
        -- direction is Target Pos to the next point on the chain
        local pointTo = pointTowards - pointFrom

        -- Gotta reverse the direction first
        -- The constraint only works if the direction is opposite
        local newLimbVector = -pointTo.Unit * limbLengthTable[i]

        -- Checks if there is a limb constraint for the current limb in the iteration
        if limbConstraintTable and limbConstraintTable[i] and limbConstraintTable[i] ~= nil then

            local limbLength = limbLengthTable[i]
            newLimbVector = limbConstraintTable[i]:ConstrainLimbVector(pointTowards, newLimbVector, limbLength)

        end
        -- constructs the new vectable
        -- Gotta make it negative though to counteract
        IteratedLimbVectorTable[i] = -newLimbVector
        vectorSumFromOrigin = vectorSumFromOrigin + IteratedLimbVectorTable[i]
    end

    self.IteratedLimbVectorTable = IteratedLimbVectorTable

end

--[[
	Does the forward chain of the FABRIK Algorithm
	Function should be called after the Backwards function in order to prevent the vector direction from changing
	Assumes vector chain direction is from endpoint to startpoint
	Returns parameters with new vector chain direction from Startpoint to EndPoint
]]
function FabrikSolver:Forwards(originCF, targetPos)

    local IteratedLimbVectorTable = self.IteratedLimbVectorTable
    local limbLengthTable = self.LimbLengthTable
    local limbConstraintTable = self.LimbConstraintTable

    local vectorSumFromOrigin = Vector3.new()
    for i = 1, #IteratedLimbVectorTable, 1 do
        -- initialize empty vector for summing
        local vectorSum = Vector3.new(0, 0, 0)

        -- Sums up the vectors in order to get the target position on the chain
        -- target position is the next joint from origin to target
        for v = i + 1, #IteratedLimbVectorTable, 1 do
            vectorSum = vectorSum + IteratedLimbVectorTable[v]
        end

        local nextJointPosition = vectorSum + targetPos
        local jointPosition = originCF.Position + vectorSumFromOrigin
        -- Gets the new direction of the new vector along the chain
        -- direction of the new vector is from origin to target
        local pointTo = nextJointPosition - jointPosition
        -- This time constraint the vector using the conical constraint function
        local newLimbVector = pointTo.Unit * limbLengthTable[i]

        -- Checks if there is a limb constraint for the current limb in the iteration
        --Also if even the table exist in the first place to avoid indexing nil value
        if limbConstraintTable and limbConstraintTable[i] and limbConstraintTable[i] ~= nil then

            local limbLength = limbLengthTable[i]
            -- Start the constraint according to the method
            newLimbVector = limbConstraintTable[i]:ConstrainLimbVector(jointPosition, newLimbVector, limbLength)

        end
        -- constructs the new vectable
        IteratedLimbVectorTable[i] = newLimbVector
        vectorSumFromOrigin = vectorSumFromOrigin + IteratedLimbVectorTable[i]
    end

    -- Change the objects self vector table
    self.IteratedLimbVectorTable = IteratedLimbVectorTable
end

--Does the CFrame rotation similar to update motors
--I plan to use this CFrame rotation to then obtain the new axis of the motors

function FabrikSolver:RotateLimb()

    -- Gets the CFrame of the Initial joint at world space
    if self.Motor6DTable[1].Part0 then
        local initialJointCFrame = self.Motor6DTable[1].Part0.CFrame * self.FirstJointC0
        
        --Vector sum for the for loop to get the series of position of the next joint based on the algorithm
        local vectorSumFromFirstJoint = Vector3.new()

        --Iterates and start rotating the limbs starting from the first joint
        for i = 1, #self.LimbVectorTable, 1 do

            --Obtains the current limb that is being worked on
            local originalVectorLimb = self.LimbVectorTable[i]
            local currentVectorLimb = self.IteratedLimbVectorTable[i]

            --Obtains the CFrame of the part0 limb of the motor6d
            local previousLimbPart = self.Motor6DTable[i].Part0
            if previousLimbPart then
                local previousLimbCF = previousLimbPart.CFrame

                -- Obtains the CFrame rotation calculation for CFrame.fromAxis
                local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)
                local dotProductAngle = limbVectorRelativeToOriginal.Unit:Dot(currentVectorLimb.Unit)
                local safetyClamp = math.clamp(dotProductAngle, -1, 1)
                local limbRotationAngle = math.acos(safetyClamp)
                local limbRotationAxis = limbVectorRelativeToOriginal:Cross(currentVectorLimb) -- obtain the rotation axis

                --Checks if the axis exists if cross product returns zero somehow
                if limbRotationAxis~=Vector3.new(0,0,0) then
                
                    --Gets the world space of the joint from the iterated limb vectors
                    if i ~= 1 then
                    vectorSumFromFirstJoint = vectorSumFromFirstJoint + self.IteratedLimbVectorTable[i-1]
                    end

                    --Gets the position of the current limb joint
                    local motorPosition = initialJointCFrame.Position + vectorSumFromFirstJoint

                    --Now adding a debug mode----------------------------------------------------------------
                    --Puts the created parts according to the motor position
                    if self.DebugMode then
                        workspace["LimbVector:"..i].Position = motorPosition
                    end
                    ----------------------------------------------------------------
                    --Obtain the CFrame operations needed to rotate the limb to the goal
                    local undoPreviousLimbCF = previousLimbCF:Inverse()*CFrame.new(motorPosition)
                    local rotateLimbCF =CFrame.fromAxisAngle(limbRotationAxis,limbRotationAngle)*CFrame.fromMatrix(Vector3.new(),previousLimbCF.RightVector,previousLimbCF.UpVector)
                    
                    local goalCF = undoPreviousLimbCF*rotateLimbCF

                    self.LimbCFrameTable[i] = goalCF

                end
            end
        end
    end
end


return FabrikSolver
