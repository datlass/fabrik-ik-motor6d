--Example script for controlling the left leg of a model in the workspace

--Get service
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

--Modules required
local IKControllerPointer = ReplicatedStorage.Source.ObjectFolder.LimbChain
local LimbChain = require(IKControllerPointer)

--RotatedRegion3 Module
local RotatedRegion3Pointer = ReplicatedStorage.Source.ObjectFolder.RotatedRegion3
local RotatedRegion3 = require(RotatedRegion3Pointer)

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

local hipMotor = lowerBody.HipBase.Hip

-- Obtain Motor6d's in left leg
local lHipToLegMotor = lowerBody.Hip.LUpperLeg
local lUpToKneeMotor = lowerBody.LeftLeg.LUpperLeg.LKnee
local lJKneeToLowMotor = lowerBody.LeftLeg.LKnee.LLowerLeg
local lLowToFeetMotor = lowerBody.LeftLeg.LLowerLeg.LFeet

-- Obtain Motor6d's in Right leg
local rHipToLegMotor = lowerBody.Hip.RUpperLeg
local rUpToKneeMotor = lowerBody.RightLeg.RUpperLeg.RKnee
local rJKneeToLowMotor = lowerBody.RightLeg.RKnee.RLowerLeg
local rLowToFeetMotor = lowerBody.RightLeg.RLowerLeg.RFeet

--Store the motor6d in table
local motorTable = {lHipToLegMotor,lUpToKneeMotor,lJKneeToLowMotor,lLowToFeetMotor}

--Store the motor6d in table
local motorRightTable = {rHipToLegMotor,rUpToKneeMotor,rJKneeToLowMotor,rLowToFeetMotor}

--Store the motor6d in table
local motorHip = {hipMotor,lHipToLegMotor}

--Initialize the left leg chain
local leftLegChain = LimbChain.new(motorTable,true)

--Initialize the right leg chain
local rightLegChain = LimbChain.new(motorRightTable,true)

--Initialize the hip nvm later
local hipChain = LimbChain.new(motorHip)


--Testing the constraint
local testBallSocketConstraint = lowerBody.Constraints.UpperLegConstraint
local upperLegBallSocketConstraint = BallSocketConstraint.new(testBallSocketConstraint,30,30)

local kneePart = lowerBody.Constraints.KneeConstraint
local lKneeHinge = BallSocketConstraint.new(kneePart,15,90)

local lLegPart = lowerBody.Constraints.LowerLegConstraint
local lLegHinge = BallSocketConstraint.new(lLegPart,10,80)

--Make the FABRIK chain not move
local rigidFeet = RigidConstraint.new(leftLegChain,4)

local limbConstraintTable = {upperLegBallSocketConstraint,lKneeHinge,lLegHinge,rigidFeet}

--Set the constraints of the object
leftLegChain:SetConstraints(limbConstraintTable)

--Repeat for the right leg

--Testing the constraint
local testBallSocketConstraint = lowerBody.Constraints.rUpperLegConstraint
local rupperLegBallSocketConstraint = BallSocketConstraint.new(testBallSocketConstraint,30,30)

local kneePart = lowerBody.Constraints.rKneeConstraint
local rKneeHinge = BallSocketConstraint.new(kneePart,15,90)

local lLegPart = lowerBody.Constraints.rLowerLegConstraint
local rLegHinge = BallSocketConstraint.new(lLegPart,10,80)

--Make the FABRIK chain not move
local rigidRightFeet = RigidConstraint.new(rightLegChain,4)


local rightLimbConstraints = {rupperLegBallSocketConstraint,rKneeHinge,rLegHinge,rigidRightFeet}

--Set the constraints of the object
rightLegChain:SetConstraints(rightLimbConstraints)

--turn on debug mode if u want
--leftLegChain:DebugModeOn()


--[[
    Then use the LimbChain object to control the motor every heartbeat
    ]]
RunService.Heartbeat:Connect(function()
        
    --The Goal position
    local goalPosition = workspace.MechLTarget.Position
    local goalRightPosition = workspace.MechRTarget.Position

    leftLegChain:IterateOnce(goalPosition,0.1)
    leftLegChain:UpdateMotors()

    rightLegChain:IterateOnce(goalRightPosition,0.1)
    rightLegChain:UpdateMotors()

    --hipChain:IterateOnce(hipGoal,0.1)
    --hipChain:UpdateMotors()

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
