-- Code for testing purposes
-- Roblox Services
local workspace = game:GetService("Workspace")
local RunService = game:GetService("RunService")
local storage = game:GetService("ReplicatedStorage")

-- Random Parts to debug position of where joints should be according to the algorithm
local part1 = game.Workspace.test1
local part2 = game.Workspace.test2
local part3 = game.Workspace.test3
local part4 = game.Workspace.test4
local vecTest = game.Workspace.VecTest

-- IK Module
local fabrik = storage.Source.IKAnimation:WaitForChild("Fabrik")
local FabrikAlgo = require(fabrik)

-- Pointers
local lowerBody = workspace.LowerBody

-- Motor6d's in left leg
local lHipToLegMotor = lowerBody.Hip.LUpperLeg
local lUpToKneeMotor = lowerBody.LeftLeg.LUpperLeg.LKnee
local lJKneeToLowMotor = lowerBody.LeftLeg.LKnee.LLowerLeg
local lLowToFeetMotor = lowerBody.LeftLeg.LLowerLeg.LFeet

-- C0 is the motor 6d position
-- C1 is the part its connected to center
-- store the original c0 position
local hipToLegStore = lHipToLegMotor.C0
local kneeToUpperLegStore = lUpToKneeMotor.C0
local kneeToLowerLegStore = lJKneeToLowMotor.C0
-- local feetStore = lLowToFeetMotor.C0

-- Debugging difference between c0 and c1 for motor6d joint instance
-- c0 is center of hip to the selectable joint
-- print(lHipToLegMotor.C0.p)
-- c1 is center of the part c0 is attached to(upperleg) towards the c0 position(hip)
-- print(lHipToLegMotor.C1.p)

-- find length of the joints
local function lenOneToTwo(partOne, partTwo)
    -- Check if its a motor6d
    if partOne:IsA("Motor6D") and partTwo:IsA("Motor6D") then
        local vecOne = partOne.C1.p
        local vecTwo = partTwo.C0.p
        local combinedVector = vecTwo - vecOne
        return combinedVector
    end
    return "Error: motor 6ds are not inserted in the function"
end
-- Obtain the vectors of the IK limbs
local vecOne = lenOneToTwo(lHipToLegMotor, lUpToKneeMotor)
local vecTwo = lenOneToTwo(lUpToKneeMotor, lJKneeToLowMotor)
local vecThree = lenOneToTwo(lJKneeToLowMotor, lLowToFeetMotor)
-- testing--it works better
local v1New = vecOne
local v2New = vecTwo
local v3New = vecThree
-- Construct the new limb vector table
local limbVectorTable = {v1New, v2New, v3New}
-- empty
local limbLengthTable = {}
for i = 1, #limbVectorTable, 1 do
    table.insert(limbLengthTable, i, limbVectorTable[i].Magnitude)
end
-- Obtain angle of joint length to where its facing
local front = Vector3.new(0, 0, -1)
local vecOneHoriz = vecOne * Vector3.new(1, 0, 1)
local vecOneVert = vecOne * Vector3.new(0, 1, 1)

local horizAngle = math.acos(front:Dot(vecOneHoriz) /
                                 (front.Magnitude * vecOneHoriz.Magnitude))
local vertAngle = math.acos(front:Dot(vecOneVert) /
                                (front.Magnitude * vecOneVert.Magnitude))

--local upperLegToKneeStore = lUpToKneeMotor.C0

RunService.Heartbeat:Connect(function()

    -- Original position and orientation of the hip in the world space
    -- Position of the hip changes so gotta put it in the heartbeat
    local hipCFrame = lowerBody.Hip.CFrame
    local upLegCFrame = lowerBody.LeftLeg.LUpperLeg.CFrame
    local kneeCFrame = lowerBody.LeftLeg.LKnee.CFrame

    --Gets the CFrame of the joint at world space
    local hipJointCFrame = lowerBody.Hip.CFrame * hipToLegStore
   -- local kneeToUpCframe = hipJointCFrame*kneeToUpperLegStore
    -- local kneeToLowCFrame = kneeToUpCframe*kneeToLowerLegStore
   
    -- the goal position
    local goalPosition = workspace.LTarget.Position

    -- goal Pos to Hip joint
    local newUpVector = hipJointCFrame.p - goalPosition

    -- Obtain vectors of where the limbs should point to
    -- local v1New,v2New,v3New= fabrikAlgo(0.1,hipJointCFrame, goalPosition, vecOne,vecTwo,vecThree)
    -- needs to stop running algorithm
    -- print(v1New,v2New,v3New)

    -- Original limb vector positons relative to their part0 CFrames
    local vectorOneRelativeToHip = hipCFrame:VectorToWorldSpace(vecOne)
    local vecTwoRel = upLegCFrame:VectorToWorldSpace(vecTwo)

    local refCF = CFrame.new(Vector3.new(), vectorOneRelativeToHip)
    -- Construct the upper leg constraint
    -- Reference to hipjointcf
    local yAxis = hipJointCFrame.UpVector
    local centerAxis = -hipJointCFrame.RightVector

    -- Reference to original vector position
    -- local yAxis = refCF.UpVector
    -- local centerAxis = refCF.LookVector
    -- widthangle,heightangle
    local angles = {80, 80}
    local upperLegConstraint = {yAxis, centerAxis, angles}

    local rotateCF = CFrame.fromAxisAngle(-hipCFrame.RightVector, math.rad(80))
    refCF = CFrame.new(Vector3.new(), vecTwoRel) * rotateCF
    yAxis = refCF.UpVector
    centerAxis = refCF.LookVector
    -- widthangle,heightangle
    angles = {25, 80}
    local kneeConstraint = {yAxis, centerAxis, angles}

    -- Tabulate all the constraints
    local limbConstraintTable = {upperLegConstraint, kneeConstraint}

    limbVectorTable = FabrikAlgo(0.1, hipJointCFrame, goalPosition,
                                 limbVectorTable, limbLengthTable)
    v1New = limbVectorTable[1]
    v2New = limbVectorTable[2]
    v3New = limbVectorTable[3]

    -- Debugging the fabrik system using test parts

    part1.Position = hipJointCFrame.p + v1New
    part2.Position = hipJointCFrame.p + v1New + v2New
    part3.Position = hipJointCFrame.p + v1New + v2New + v3New

    -- obtains a vector always at knee at original position
   local vectorToCFrame = CFrame.new(Vector3.new(),vecOne)
   local rotationOffset = (hipCFrame-hipCFrame.p)
   local lookRelativeToHip = rotationOffset*vectorToCFrame
   local vectorOneRelativeToHip = lookRelativeToHip.LookVector*limbLengthTable[1]

    -- Obtain angle between new hip-upperleg vector and its original position
    local rotateUpperLegAngle = math.acos(vectorOneRelativeToHip.Unit:Dot(v1New.Unit))
    local rotationAxisUpperLeg = vectorOneRelativeToHip:Cross(v1New) -- obtain the rotation axis

    -- repeat for the knee joint
    local vecTwoRel = upLegCFrame:VectorToWorldSpace(vecTwo)
    -- something wrong when the angle goes more than 90 degrees
    local rotateKneeAngle = math.acos(vecTwoRel.Unit:Dot(v2New.Unit))
    local rotationAxisKnee = vecTwoRel:Cross(v2New) -- obtain the rotation axis

    vecTest.Position = hipJointCFrame.p + v1New + vecTwoRel

    -- repeat for the lower Leg 
    -- CFrame relativity broke fix:
    -- kneeCFrame
    -- lUpToKneeMotor.C0
    local vecThreeRel = kneeCFrame:VectorToWorldSpace(vecThree)
    local rotateLowerLegAngle = math.acos( vecThreeRel:Dot(v3New) / (vecThreeRel.Magnitude * v3New.Magnitude))
    local rotationAxisLowerLeg = vecThreeRel:Cross(v3New) -- obtain the rotation axis

    --Position of the original position
    part4.Position = hipJointCFrame.p+v1New+v2New+vecThreeRel

    local empty = Vector3.new()

    local upperLegRightVector = v1New:Cross(lowerBody.Hip.CFrame.LookVector)

    --Change the upper leg position through the hip motor
    local upperLegPos = hipJointCFrame.p
    local undoPreviousLimbCF = hipCFrame:Inverse()*CFrame.new(upperLegPos)
    local rotateLimbCF =CFrame.fromAxisAngle(rotationAxisUpperLeg,rotateUpperLegAngle)*CFrame.new(empty,hipCFrame.LookVector)
    lHipToLegMotor.C0 = undoPreviousLimbCF*rotateLimbCF

    --*CFrame.fromAxisAngle(rotationAxisUpperLeg,rotateUpperLegAngle)

    --Change the knee through the upperleg motor
    local kneePos = hipJointCFrame.p + v1New    
    local undoPreviousLimbCF = upLegCFrame:Inverse()*CFrame.new(kneePos)
    local rotateLimbCF =CFrame.fromAxisAngle(rotationAxisKnee,rotateKneeAngle)*CFrame.new(empty,upLegCFrame.LookVector)
    lUpToKneeMotor.C0 = undoPreviousLimbCF*rotateLimbCF

    --Change the lowerleg through the knee motor
    local lowerLegPos = hipJointCFrame.p + v1New + v2New
    local lowerLegRightVector = -Vector3.new(0, 1, 0):Cross(v3New)
    local newLowLegCF = kneeCFrame:Inverse() *
                            CFrame.fromMatrix(lowerLegPos, lowerLegRightVector,
                                              -v3New)
    local undoPreviousLimbCF = kneeCFrame:Inverse()*CFrame.new(lowerLegPos)
    local rotateLimbCF =CFrame.fromAxisAngle(rotationAxisLowerLeg,rotateLowerLegAngle)*CFrame.new(empty,kneeCFrame.LookVector)
                                          --*CFrame.new(empty,kneeCFrame.LookVector)
     lJKneeToLowMotor.C0 = undoPreviousLimbCF*rotateLimbCF
   -- lJKneeToLowMotor.C0 = newLowLegCF
end)
