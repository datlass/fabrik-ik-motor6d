-- Get Roblox Services
local ReplicatedStorage = game:GetService("ReplicatedStorage")

-- Import the Fabrik Module
local fabrik = ReplicatedStorage.Source.IKAnimation:WaitForChild("Fabrik")
local FabrikAlgo = require(fabrik)


-- Import the Fabrik Module Object
--test
local fabrikSolver = ReplicatedStorage.Source.ObjectFolder:WaitForChild("FabrikSolver")
local FabrikSolver = require(fabrikSolver)

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)
local LimbChain = Object.new("LimbChain")

--[[
    Initializes the limb chain object
    Calculates the limbs from joint to joint as a vector
    And also measures the limb's length
]]
function LimbChain.new(Motor6DTable,IncludeAppendage,LimbConstraintTable)
    --Does the meta table stuff
    local obj = LimbChain:make()

    obj.IncludeAppendage = IncludeAppendage

    obj.Motor6DTable = Motor6DTable
    obj.FirstJointC0 = Motor6DTable[1].C0
    -- initialize LimbVectorTable to store the limb vectors and stores it into the object
    local LimbVectorTable = {}
    local IteratedLimbVectorTable = {}
    for i = 1, #Motor6DTable - 1, 1 do
        --print("motorOne: ", Motor6DTable[i].Name,"motorTwo: ",Motor6DTable[i+1].Name)
        --print(LimbChain:JointOneToTwoVector(Motor6DTable[i], Motor6DTable[i + 1]))
        local currentVectorStore = LimbChain:JointOneToTwoVector(Motor6DTable[i], Motor6DTable[i + 1])
        LimbVectorTable[#LimbVectorTable + 1] = currentVectorStore
        IteratedLimbVectorTable[#IteratedLimbVectorTable + 1] = currentVectorStore
    end

    --Checks if this bool is true then adds it into the vector limb
    if IncludeAppendage == true then
        local lastMotor = Motor6DTable[#Motor6DTable]
        --print(lastMotor)
        --local test = -lastMotor.C1.Position
        --This is more accurate than getting C1 position for some reason
        local ExtraLimbVector = lastMotor.Part1.Position-(lastMotor.Part0.Position+lastMotor.C0.Position)

        LimbVectorTable[#LimbVectorTable + 1] = ExtraLimbVector
        IteratedLimbVectorTable[#IteratedLimbVectorTable + 1] = ExtraLimbVector

        obj.ExtraLimbVector = ExtraLimbVector
    end

    obj.LimbVectorTable = LimbVectorTable
    -- Stores the limb vector table in the iterated version
    obj.IteratedLimbVectorTable = IteratedLimbVectorTable

    -- Finds the length of the limb vectors and puts them in a table for the FABRIK algorithm
    -- Important as it reduces .Magnitude calls therefore reduces Activity
    local LimbLengthTable = {}
    for i = 1, #LimbVectorTable, 1 do
        LimbLengthTable[#LimbLengthTable+1] = LimbVectorTable[i].Magnitude
    end
    
    obj.LimbLengthTable = LimbLengthTable

    --Store the constraint table
    obj.LimbConstraintTable = LimbConstraintTable
    --Once the limb vectors are initialized store them in a FabrikSolver object which does the Fabrik
    local LimbFabrikSolver = FabrikSolver.new(IteratedLimbVectorTable,LimbLengthTable,LimbConstraintTable)
    obj.LimbFabrikSolver = LimbFabrikSolver


    return obj
end
--[[
    Function that limb chain has to calculate vector limbs
    Input 2 Motor6d
    Returns a vector from motorOne to motorTwo  joint
    Always constant based on the c0 and c1 Position of the motors
]]
function LimbChain:JointOneToTwoVector(motorOne, motorTwo)
    -- Check if its a motor6d
    if motorOne:IsA("Motor6D") and motorTwo:IsA("Motor6D") then
        local vecOne = motorOne.C1.Position
        local vecTwo = motorTwo.C0.Position
        local combinedVector = vecTwo - vecOne
        return combinedVector
    else
        return "Error: motor 6ds are not inserted in the function"
    end
end
--[[
    Function that executes 1 iteration of the Fabrik Algorithm towards the target position
    Deprecated method uses old IK module which does some inefficient stuff
    (like continually summing up to find the max length every iteration)
]]
function LimbChain:Iterate(tolerance, targetPosition,limbConstraintTable)

    -- Gets the CFrame of the first joint at world space
    local originJointCF = self.Motor6DTable[1].Parent.CFrame * self.FirstJointC0

    --Performs the iteration on the LimbChain object IteratedLimbVectorTable and rewrites it
    --Recursive function
    self.IteratedLimbVectorTable = FabrikAlgo(tolerance, originJointCF, targetPosition, self.IteratedLimbVectorTable, self.LimbLengthTable,limbConstraintTable)
                                              

end

--[[
    Newer iteration method that uses the Fabrik Object instead of a module
    The object stores its own
]]
function LimbChain:IterateOnce(targetPosition,tolerance)

    -- Gets the CFrame of the first joint at world space
    local originJointCF = self.Motor6DTable[1].Parent.CFrame * self.FirstJointC0

    --Does the fabrik iteration once if not in goal
    self.IteratedLimbVectorTable = self.LimbFabrikSolver:IterateOnce(originJointCF,targetPosition, tolerance)
                                              

end

function LimbChain:IterateUntilGoal(targetPosition,tolerance,InputtedMaxBreakCount)

    -- Gets the CFrame of the first joint at world space
    local originJointCF = self.Motor6DTable[1].Parent.CFrame * self.FirstJointC0

    --Does the fabrik iteration until goal
    self.IteratedLimbVectorTable = self.LimbFabrikSolver:IterateUntilGoal(originJointCF,targetPosition, tolerance,InputtedMaxBreakCount)
                                              

end
--[[
    Function that rotates the motors to match the algorithm
    Operates by changing the motor's C0 Position to the goal CFrame
    Optional Parameter floorNormal to make feet upright to the floor its being raycasted to
    Feet implementation is real buggy tho and is not automatic implementation
    And it only works for my mech model
]]
function LimbChain:UpdateMotors(floorNormal)

    -- Gets the CFrame of the Initial joint at world space
    local initialJointCFrame = self.Motor6DTable[1].Parent.CFrame * self.FirstJointC0

    --Vector sum for the for loop to get the series of position of the next joint based on the algorithm
    local vectorSumFromFirstJoint = Vector3.new()

    local iterateUntil = #self.LimbVectorTable

    --Iterates and start rotating the limbs starting from the first joint
    for i = 1, iterateUntil, 1 do

        --Obtains the current limb that is being worked on
        local originalVectorLimb = self.LimbVectorTable[i]
        local currentVectorLimb = self.IteratedLimbVectorTable[i]

        --Obtains the CFrame of the part0 limb of the motor6d
        local previousLimbPart = self.Motor6DTable[i].Parent
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

        --Obtain the CFrame operations needed to rotate the limb to the goal
        local undoPreviousLimbCF = previousLimbCF:Inverse()*CFrame.new(motorPosition)
        local rotateLimbCF =CFrame.fromAxisAngle(limbRotationAxis,limbRotationAngle)*CFrame.fromMatrix(Vector3.new(),previousLimbCF.RightVector,previousLimbCF.UpVector)
        
        --Changes the current motor6d through c0
        self.Motor6DTable[i].C0 = undoPreviousLimbCF*rotateLimbCF

            --Testing to keep foot appendage upright
            --Seems to work for now to make the feet look down visually
            --doesn't adhere to constraints but it works for now
            --I really need help making the feet match the surface
            if self.IncludeAppendage == true and i == iterateUntil then
                local previousLimbPart = self.Motor6DTable[#self.Motor6DTable].Parent
                local previousLimbCF = previousLimbPart.CFrame
                local empty = Vector3.new()
                --self.Motor6DTable[#self.Motor6DTable].C0 = CFrame.new()

                --Make feet point upright to floor
                --If not inputted then make feet points up towards sky
                local upright = floorNormal
                if upright == nil then
                    upright = Vector3.new(0,1,0)
                end

                --Obtain the CFrame operations needed to rotate the limb to the goal
                local undoPreviousLimbCF = previousLimbCF:Inverse()*CFrame.new(motorPosition)
                local rotateLimbCF =CFrame.fromMatrix(empty,previousLimbCF.RightVector,upright)
            
                --Changes the current motor6d through c0
                self.Motor6DTable[i].C0 = undoPreviousLimbCF*rotateLimbCF
    
            end
        end

    
    end



end


--Prints  the limb vector and iterated limb vector
--For debugging not needed now
function LimbChain:PrintLimbVectors()

    for i=1,#self.LimbVectorTable,1 do
        --print("Limbvector table i:",i," Vector:",self.LimbVectorTable[i])
    end
    for i=1,#self.IteratedLimbVectorTable,1 do
        print("Iterated self table i:",i," Vector:",self.IteratedLimbVectorTable[i])
    end

end

--Gets the original vector limb direction relative to the part
function LimbChain:GetOriginalLimbDirection(limbVectorNumber)

    local i = limbVectorNumber

    --Obtains the current limb that is being worked on
    local originalVectorLimb = self.LimbVectorTable[i]

    --print(self.Motor6DTable[i].Parent)
    --Obtains the CFrame of the part0 limb of the motor6d
    local previousLimbPart = self.Motor6DTable[i].Parent
    local previousLimbCF = previousLimbPart.CFrame

    -- Obtains the vector relative to the previous part0
    local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)
    
    return limbVectorRelativeToOriginal

end

--Testing for feet
function LimbChain:GetOriginalFeetLimb()

    --Obtains the current limb that is being worked on
    local originalVectorLimb = self.extraLimbVector

    --Obtains the CFrame of the part0 limb of the motor6d
    local previousLimbPart = self.Motor6DTable[#self.Motor6DTable].Parent
    local previousLimbCF = previousLimbPart.CFrame

    -- Obtains the vector relative to the previous part0
    local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)

    return limbVectorRelativeToOriginal

end

function LimbChain:SetConstraints(LimbConstraintTable)

    --Changes the constraint table and the fabrik solvers as well
    self.LimbConstraintTable = LimbConstraintTable
    self.LimbFabrikSolver.LimbConstraintTable = LimbConstraintTable


end

return LimbChain
