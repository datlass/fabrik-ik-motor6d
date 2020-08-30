-- Get Roblox Services
local ReplicatedStorage = game:GetService("ReplicatedStorage")

-- Import the Fabrik Module
local fabrik = ReplicatedStorage.Source.IKAnimation:WaitForChild("Fabrik")
local FabrikAlgo = require(fabrik)

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)
local limbChain = Object.new("limbChain")

--empty vector 3

local emptyVector = Vector3.new(0, 0, 0)

--[[
	Initializes the limb chain object
]]
function limbChain.new(Motor6DTable)
    --Does the meta table stuff
    local obj = limbChain:make()


    obj.Motor6DTable = Motor6DTable
    obj.FirstJointC0 = Motor6DTable[1].C0
    -- initialize LimbVectorTable to store the limb vectors and stores it into the object
    local LimbVectorTable = {}
    local IteratedLimbVectorTable = {}
    for i = 1, #Motor6DTable - 1, 1 do
        --print("motorOne: ", Motor6DTable[i].Name,"motorTwo: ",Motor6DTable[i+1].Name)
        --print(limbChain:JointOneToTwoVector(Motor6DTable[i], Motor6DTable[i + 1]))
        local currentVectorStore = limbChain:JointOneToTwoVector(Motor6DTable[i], Motor6DTable[i + 1])
        LimbVectorTable[#LimbVectorTable + 1] = currentVectorStore
        IteratedLimbVectorTable[#IteratedLimbVectorTable + 1] = currentVectorStore
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

    return obj
end
-- Function that limb chain has to calculate vector limbs
function limbChain:JointOneToTwoVector(motorOne, motorTwo)
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
-- Function that executes 1 iteration of the Fabrik Algorithm
-- Backwards or forwards
function limbChain:Iterate(tolerance, targetPosition)

    -- Gets the CFrame of the first joint at world space
    local originJointCF = self.Motor6DTable[1].Parent.CFrame * self.FirstJointC0
    --print(self.IteratedLimbVectorTable[1])
    self.IteratedLimbVectorTable = FabrikAlgo(tolerance, originJointCF, targetPosition, self.IteratedLimbVectorTable, self.LimbLengthTable)
                                              

end

function limbChain:UpdateMotors()

    -- Gets the CFrame of the joint at world space
    local initialJointCFrame = self.Motor6DTable[1].Parent.CFrame * self.FirstJointC0


    local vectorSumFromFirstJoint = Vector3.new()

    --Iterates from
    --Hip i =1
    --LUpperLeg i=2
    --LKnee i=3
    for i = 1, #self.LimbVectorTable, 1 do

        local originalVectorLimb = self.LimbVectorTable[i]
        local currentVectorLimb = self.IteratedLimbVectorTable[i]
        --Seems working fine
        --print("Original: ",originalVectorLimb,"Current: ",currentVectorLimb)
        --local Part0 Limb CFrame
        local previousLimbPart = self.Motor6DTable[i].Parent
        local previousLimbCF = previousLimbPart.CFrame

        --print("Name: ",self.Motor6DTable[i].Parent.Name,"Index: ",i)
        --print(previousLimbCF)

        -- For the current joint
        local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)
        local limbRotationAngle = math.acos(limbVectorRelativeToOriginal.Unit:Dot(currentVectorLimb.Unit))
        local limbRotationAxis = limbVectorRelativeToOriginal:Cross(currentVectorLimb) -- obtain the rotation axis
        
        --print(limbVectorRelativeToOriginal)
        --print("Angle: ", limbRotationAngle,"Axis: ",limbRotationAxis)
       
        if i ~= 1 then
        vectorSumFromFirstJoint = vectorSumFromFirstJoint + self.IteratedLimbVectorTable[i-1]
        end

        --Gets the position of the limb
        local motorPosition = initialJointCFrame.Position + vectorSumFromFirstJoint

        --Obtain the CFrame operations needed to rotate the limb to the goal
        local undoPreviousLimbCF = previousLimbCF:Inverse()*CFrame.new(motorPosition)
        local rotateLimbCF =CFrame.fromAxisAngle(limbRotationAxis,limbRotationAngle)*CFrame.fromMatrix(Vector3.new(),previousLimbCF.RightVector,previousLimbCF.UpVector)
        
        --Changes the current motor6d through c0
        self.Motor6DTable[i].C0 = undoPreviousLimbCF*rotateLimbCF
        
    end

end

function limbChain:PrintLimbVectors()

    for i=1,#self.LimbVectorTable,1 do
        --print("Limbvector table i:",i," Vector:",self.LimbVectorTable[i])
    end
    for i=1,#self.IteratedLimbVectorTable,1 do
        print("Iterated self table i:",i," Vector:",self.IteratedLimbVectorTable[i])
    end

end

return limbChain