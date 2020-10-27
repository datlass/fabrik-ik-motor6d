-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

local FabrikSolver = Object.new("FabrikSolver")

local function shallowCopy(original)
	local copy = {}
	for key, value in pairs(original) do
		copy[key] = value
	end
	return copy
end

function FabrikSolver.new(LimbVectorTable, LimbLengthTable, LimbConstraintTable,LimbChain)
    local obj = FabrikSolver:make()

    obj.LimbChain = LimbChain
    -- Initialize the object class
    obj.LimbVectorTable = LimbVectorTable

    obj.LimbLengthTable = LimbLengthTable

    obj.LimbConstraintTable = LimbConstraintTable

    obj.LimbCFrameTable = {}

    obj.OriginalLimbVectorTable = shallowCopy(LimbVectorTable)

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

    -- initialize measure feet to where it should be in the world position
    local vectorSum = Vector3.new(0, 0, 0)
    for i = 1, #self.LimbVectorTable, 1 do
        vectorSum = vectorSum + self.LimbVectorTable[i]
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
        self:ConstrainLimbs(originCF)

        return self.LimbVectorTable

    else
        -- Limb is within tolerance/already reached goal so don't do anything
        -- Do backwards on itself first then forwards
        self:Backwards(originCF, targetPosition)
        self:Forwards(originCF, targetPosition)
        self:ConstrainLimbs(originCF)

        return self.LimbVectorTable
    end

end

function FabrikSolver:IterateUntilGoal(originCF, targetPosition, tolerance,
                                       InputtedMaxBreakCount)

    -- initialize measure feet to where it should be in the world position
    local vectorSum = Vector3.new(0, 0, 0)
    for i = 1, #self.LimbVectorTable, 1 do
        vectorSum = vectorSum + self.LimbVectorTable[i]
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
        for i = 1, #self.LimbVectorTable, 1 do
          vectorSum = vectorSum + self.LimbVectorTable[i]
        end
        local feetJoint = originCF.Position + vectorSum
        local feetToTarget = targetPosition - feetJoint
        distanceToGoal = feetToTarget.Magnitude
        
        --Counts the amount of iterations, if impossible to solve stops after max default at 10 iterations
        bcount += 1
        if bcount > maxBreakCount then 
            --print("bcount:", bcount,"failed to reach goal: ", distanceToGoal)
            return self.LimbVectorTable 
        end

    end
    --Iterate once in case
    self:Backwards(originCF, targetPosition)
    self:Forwards(originCF, targetPosition)

    -- Limb is within tolerance/already reached goal so don't do anything
    --print("bcount:", bcount,"Reached goal: ", distanceToGoal)
    return self.LimbVectorTable

end

--[[
    Constraining the forwards operation only makes it weird I dont think it simply does enough
    So constraints are also applied to backwards now
    Now is generalized with less parameters thanks to meta table objects storing the properties
]]
function FabrikSolver:Backwards(originCF, targetPos)

    -- Transporting from module scrip to class so gotta do this
    local limbVectorTable = self.LimbVectorTable
    local limbLengthTable = self.LimbLengthTable
    local limbConstraintTable = self.LimbConstraintTable

    local vectorSumFromOrigin = Vector3.new()
    -- Iterate through all the limb vectors and performs the backwards operation
    for i = #limbVectorTable, 1, -1 do
        local vectorSum = Vector3.new(0, 0, 0)

        for v = 1, i - 1, 1 do 
            vectorSum = vectorSum + limbVectorTable[v] 
        end

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
            -- Start the constraint according to the method
            -- print(limbConstraintTable[i])
            --newLimbVector = limbConstraintTable[i]:ConstrainLimbVector(pointTowards, newLimbVector, limbLength)
            -- print("Index: ",i,"Vector: ",newLimbVector)
        end
        -- constructs the new vectable
        -- Gotta make it negative though to counteract
        limbVectorTable[i] = -newLimbVector
        vectorSumFromOrigin = vectorSumFromOrigin + limbVectorTable[i]
    end

    -- Change the objects self vector table
    self.LimbVectorTable = limbVectorTable

end

--[[
	Does the forward chain of the FABRIK Algorithm
	Function should be called after the Backwards function in order to prevent the vector direction from changing
	Assumes vector chain is from endpoint to startpoint
	Returns parameters with new vector chain direction from Startpoint to EndPoint
]]
function FabrikSolver:Forwards(originCF, targetPos)

    local limbVectorTable = self.LimbVectorTable
    local limbLengthTable = self.LimbLengthTable
    local limbConstraintTable = self.LimbConstraintTable

    local vectorSumFromOrigin = Vector3.new()
    for i = 1, #limbVectorTable, 1 do
        -- initialize empty vector for summing
        local vectorSum = Vector3.new(0, 0, 0)

        -- Sums up the vectors in order to get the target position on the chain
        -- target position is the next joint from origin to target
        for v = i + 1, #limbVectorTable, 1 do
            vectorSum = vectorSum + limbVectorTable[v]
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

            --newLimbVector = limbConstraintTable[i]:ConstrainLimbVector(jointPosition, newLimbVector, limbLength)

        end
        -- constructs the new vectable
        limbVectorTable[i] = newLimbVector
        vectorSumFromOrigin = vectorSumFromOrigin + limbVectorTable[i]
    end

    -- Change the objects self vector table
    self.LimbVectorTable = limbVectorTable
end


--goes fowards from origin cf joint to current end joint
--start point to end point
function FabrikSolver:ConstrainLimbs(originCF)

    local limbVectorTable = self.LimbVectorTable
    local limbLengthTable = self.LimbLengthTable
    local limbConstraintTable = self.LimbConstraintTable

    for i = 1, #limbVectorTable, 1 do
        -- initialize empty vector for summing
        local vectorSum = Vector3.new(0, 0, 0)

        -- Sums up the vectors in order to get the target position on the chain
        -- target position is the next joint from origin to target
        local currentLimbVector = limbVectorTable[i]
        if i ~= 1 then
            for v = 1, i-1, 1 do 
                vectorSum = vectorSum + limbVectorTable[v] 
            end
        end

        local currentJointPosition = vectorSum + originCF.Position

        local nextJointPosition = currentJointPosition + currentLimbVector
        local midPointPosition = currentJointPosition + currentLimbVector/2
        local LimbVectorCFrame
        --assumes up axis is up
        --doesn't work rotation is too random
        if i == 1 then
            --LimbVectorCFrame = CFrame.lookAt(currentJointPosition,nextJointPosition,originCF.RightVector)
        else
            --LimbVectorCFrame = CFrame.lookAt(currentJointPosition,nextJointPosition,-currentLimbVector)
        end
        local originalVectorLimb =self.OriginalLimbVectorTable[i]
        local previousLimbCF
        if i == 1 then
            local part0CF = originCF*self.FirstJointC0:Inverse()
            previousLimbCF = part0CF
        else
            local rotOnly = self.LimbCFrameTable[i-1]-self.LimbCFrameTable[i-1].Position
            previousLimbCF = CFrame.new(midPointPosition)*rotOnly
        end
        local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)

        --angle finding and rotation axis
        local dotProductAngle = limbVectorRelativeToOriginal.Unit:Dot(currentLimbVector.Unit)
        local safetyClamp = math.clamp(dotProductAngle, -1, 1)
        local limbRotationAngle = math.acos(safetyClamp)
        local limbRotationAxis = limbVectorRelativeToOriginal:Cross(currentLimbVector) -- obtain the rotation axis

        --new testing for rotating the rectangle
        --local goalCF = previousLimbCF*CFrame.fromAxisAngle(limbRotationAxis,limbRotationAngle)
        local rotOnly = previousLimbCF-previousLimbCF.p
        local undoPreviousLimbCF = CFrame.new(currentJointPosition)
        local rotateLimbCF =CFrame.fromAxisAngle(limbRotationAxis,limbRotationAngle)*rotOnly
        
        local goalCF = undoPreviousLimbCF*rotateLimbCF

        LimbVectorCFrame = goalCF

        self.LimbCFrameTable[i] = LimbVectorCFrame

        self:DebugLimbs(i,LimbVectorCFrame,midPointPosition)

        if limbConstraintTable and limbConstraintTable[i] and limbConstraintTable[i] ~= nil then

            local limbLength = limbLengthTable[i]
            -- Start the constraint according to the method
            if i == 1 then
                currentLimbVector = limbConstraintTable[i]:ConstrainLimbVector(currentJointPosition, currentLimbVector, limbLength,originCF)
            else
                currentLimbVector = limbConstraintTable[i]:ConstrainLimbVector(currentJointPosition, currentLimbVector, limbLength,self.LimbCFrameTable[i-1])
            end

        end

        --self.LimbVectorTable[i] = currentLimbVector

    end

end


function FabrikSolver:DebugLimbs(index,limbCFrame,midPointPosition)
    --initizalize first
    if not self.Initialized then
        self.Initialized = true
        local LimbPartTable = {}
        for i, v in pairs(self.LimbVectorTable) do
            local LimbPart = Instance.new("WedgePart")
            LimbPart.BrickColor = BrickColor.random()
            LimbPart.Name = "LimbPart"..i
            LimbPart.Anchored = true
            LimbPart.CanCollide = false
            LimbPart.Size = Vector3.new(0.5,1,self.LimbLengthTable[i])
            LimbPartTable[i] = LimbPart
            LimbPart.Parent = workspace
        end
        self.LimbPartTable=LimbPartTable
    else
        local positionCF = CFrame.new(midPointPosition)
        local rotOnly = limbCFrame-limbCFrame.p
        self.LimbPartTable[index].CFrame = positionCF*rotOnly*CFrame.new(0,0,-self.LimbLengthTable[index]/2)

    end
end

return FabrikSolver
