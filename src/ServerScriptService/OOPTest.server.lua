--Example script for controlling the left leg of a model in the workspace

--Get service
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

--Modules required

--Limb chain object
local IKControllerPointer = ReplicatedStorage.Source.ObjectFolder.limbChain
local limbChain = require(IKControllerPointer)

--RotatedRegion3 Module
local RotatedRegion3Pointer = ReplicatedStorage.Source.ObjectFolder.RotatedRegion3
local RotatedRegion3 = require(RotatedRegion3Pointer)
--It works time

-------------------Import all the Constraints Types-----------------

--BallSocketConstraint
local BallSocketConstraintPointer = ReplicatedStorage.Source.ObjectFolder.ConstraintTypes.BallSocketConstraint
local BallSocketConstraint = require(BallSocketConstraintPointer)


--HingeConstraint
local HingeConstraintPointer = ReplicatedStorage.Source.ObjectFolder.ConstraintTypes.HingeConstraint
local HingeConstraint = require(HingeConstraintPointer)

--Rigid Constraint
local RigidConstraintPointer = ReplicatedStorage.Source.ObjectFolder.ConstraintTypes.RigidConstraint
local RigidConstraint = require(RigidConstraintPointer)

----------------------------------------------------------------


-- Pointers
local lowerBody = workspace.LowerBody

-- Obtain Motor6d's in left leg
local lHipToLegMotor = lowerBody.Hip.LUpperLeg
local lUpToKneeMotor = lowerBody.LeftLeg.LUpperLeg.LKnee
local lJKneeToLowMotor = lowerBody.LeftLeg.LKnee.LLowerLeg
local lLowToFeetMotor = lowerBody.LeftLeg.LLowerLeg.LFeet

--Store the motor6d in table
local motorTable = {lHipToLegMotor,lUpToKneeMotor,lJKneeToLowMotor,lLowToFeetMotor}


local lowerBody = workspace.LowerBody.Hip

--Initialize the left leg chain
local leftLegChain = LimbChain.new(motorTable,true)

--Testing the constraint
local testBallSocketConstraint = lowerBody.Constraints.UpperLegConstraint
local upperLegBallSocketConstraint = BallSocketConstraint.new(testBallSocketConstraint,80,80)

local kneePart = lowerBody.Constraints.KneeConstraint
local lKneeHinge = HingeConstraint.new(kneePart,30,90)
local lKneeHinge = BallSocketConstraint.new(kneePart,15,60)

local lLegPart = lowerBody.Constraints.LowerLegConstraint
local lLegHinge = HingeConstraint.new(lLegPart,90,180)
local lLegHinge = BallSocketConstraint.new(lLegPart,15,80)

--Make the FABRIK chain not move
local rigidFeet = RigidConstraint.new(leftLegChain,4)

local limbConstraintTable = {upperLegBallSocketConstraint,lKneeHinge,lLegHinge,rigidFeet}

--Set the constraints of the object
leftLegChain:SetConstraints(limbConstraintTable)



--[[
    Then use the LimbChain object to control the motor every heartbeat
    ]]
RunService.Heartbeat:Connect(function()
        
    --The Goal position
    local goalPosition = workspace.LTarget.Position

    leftLegChain:IterateOnce(goalPosition,0.1)
    leftLegChain:UpdateMotors()

end)



--[[
    
--Moves position back and forth
local back = Vector3.new(0,0,15)
local goalPosition = workspace.LTarget.Position

for i=1,1000,1 do
    
    workspace.LTarget.Position = workspace.LTarget.Position+back
    leftLegChain:IterateUntilGoal(workspace.LTarget.Position,0.1,20)
    leftLegChain:UpdateMotors()
    wait(1)
    workspace.LTarget.Position = workspace.LTarget.Position-back
    leftLegChain:IterateUntilGoal(workspace.LTarget.Position,0.1,20)
    leftLegChain:UpdateMotors()
    wait(1)
end
]]


--[[
for i=1,10000,1 do
leftLegChain:IterateUntilGoal(workspace.LTarget.Position,0.1,15)
leftLegChain:UpdateMotors()
wait(1)
end
]]
