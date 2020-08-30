-- Get Roblox Services
local ReplicatedStorage = game:GetService("ReplicatedStorage")

-- Import the Fabrik Module
local fabrik = ReplicatedStorage.Source.IKAnimation:WaitForChild("Fabrik")
local FabrikAlgo = require(fabrik)

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)
local limbChain = Object.new("limbChain")

--[[
    Initializes the limb chain object
    Calculates the limbs from joint to joint as a vector
    And also measures the limb's length
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
--[[
    Function that limb chain has to calculate vector limbs
    Input 2 Motor6d
    Returns a vector from motorOne to motorTwo  joint
    Always constant based on the c0 and c1 Position of the motors
]]
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
--[[
    Function that executes 1 iteration of the Fabrik Algorithm towards the target position
]]
function limbChain:Iterate(tolerance, targetPosition)

    -- Gets the CFrame of the first joint at world space
    local originJointCF = self.Motor6DTable[1].Parent.CFrame * self.FirstJointC0

    --Performs the iteration on the limbChain object IteratedLimbVectorTable and rewrites it
    --Recursive function
    self.IteratedLimbVectorTable = FabrikAlgo(tolerance, originJointCF, targetPosition, self.IteratedLimbVectorTable, self.LimbLengthTable)
                                              

end
--[[
    Function that rotates the motors to match the algorithm
    Operates by changing the motor's C0 Position to the goal CFrame
]]
function limbChain:UpdateMotors()

    -- Gets the CFrame of the Initial joint at world space
    local initialJointCFrame = self.Motor6DTable[1].Parent.CFrame * self.FirstJointC0

    --Vector sum for the for loop to get the series of position of the next joint based on the algorithm
    local vectorSumFromFirstJoint = Vector3.new()

    --Iterates and start rotating the limbs starting from the first joint
    for i = 1, #self.LimbVectorTable, 1 do

        --Obtains the current limb that is being worked on
        local originalVectorLimb = self.LimbVectorTable[i]
        local currentVectorLimb = self.IteratedLimbVectorTable[i]

        --Obtains the CFrame of the part0 limb of the motor6d
        local previousLimbPart = self.Motor6DTable[i].Parent
        local previousLimbCF = previousLimbPart.CFrame

        -- Obtains the CFrame rotation calculation for CFrame.fromAxis
        local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)
        local limbRotationAngle = math.acos(limbVectorRelativeToOriginal.Unit:Dot(currentVectorLimb.Unit))
        local limbRotationAxis = limbVectorRelativeToOriginal:Cross(currentVectorLimb) -- obtain the rotation axis
        
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
        
    end

end


--Prints  the limb vector and iterated limb vector
--For debugging not needed now
function limbChain:PrintLimbVectors()

    for i=1,#self.LimbVectorTable,1 do
        --print("Limbvector table i:",i," Vector:",self.LimbVectorTable[i])
    end
    for i=1,#self.IteratedLimbVectorTable,1 do
        print("Iterated self table i:",i," Vector:",self.IteratedLimbVectorTable[i])
    end

end

return limbChain