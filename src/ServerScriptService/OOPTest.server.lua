--This script is supposed to replace MechIKServer with less code and make it easier to understand
--with less code in it

--Get service
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

--Modules required
local IKControllerPointer = ReplicatedStorage.Source.ObjectFolder.limbChain
local limbChain = require(IKControllerPointer)

--
-- Pointers
local lowerBody = workspace.LowerBody

-- Obtain Motor6d's in left leg
local lHipToLegMotor = lowerBody.Hip.LUpperLeg
local lUpToKneeMotor = lowerBody.LeftLeg.LUpperLeg.LKnee
local lJKneeToLowMotor = lowerBody.LeftLeg.LKnee.LLowerLeg
local lLowToFeetMotor = lowerBody.LeftLeg.LLowerLeg.LFeet

--Store the motor6d in table
local motorTable = {lHipToLegMotor,lUpToKneeMotor,lJKneeToLowMotor,lLowToFeetMotor}
local leftLegChain = limbChain.new(motorTable)

for i=1,#leftLegChain.LimbVectorTable,1 do
--print("Limbvector table i:",i," Vector:",leftLegChain.LimbVectorTable[i])
end

for i=1,#leftLegChain.LimbLengthTable,1 do
    --print("Limb length of index",i," length:",leftLegChain.LimbLengthTable[i])
end

RunService.Heartbeat:Connect(function()
        
    -- the goal position
    local goalPosition = workspace.LTarget.Position

    --leftLegChain:PrintLimbVectors()
    leftLegChain:Iterate(0.1,goalPosition)
    leftLegChain:UpdateMotors()

end)
