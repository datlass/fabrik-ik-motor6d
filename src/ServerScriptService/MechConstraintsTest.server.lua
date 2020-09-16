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

--Initialize the left leg chain
local leftLegChain = LimbChain.new(motorTable)

--Initialize the right leg chain
local rightLegChain = LimbChain.new(motorRightTable)

--Testing the constraint
local testBallSocketConstraint = lowerBody.Constraints.UpperLegConstraint
local upperLegBallSocketConstraint = BallSocketConstraint.new(testBallSocketConstraint,30,30)

local kneePart = lowerBody.Constraints.KneeConstraint
local lKneeBallSocket = BallSocketConstraint.new(kneePart,20,89)

local lLegPart = lowerBody.Constraints.LowerLegConstraint
local lLegBallSocket = BallSocketConstraint.new(lLegPart,20,89)

--[[
    Create the alternative constraints which uses hinge
    More restrictive and glitchy close to original joint but better fitting and looks nicer visually
]]
local upperLegBallSocketConstraintAlternative = BallSocketConstraint.new(testBallSocketConstraint,40,40)
local lKneeHinge = HingeConstraint.new(kneePart,90,90)
local lLegHinge = HingeConstraint.new(lLegPart,90,90)


--Set up two constraint tables to allow
local leftLegConstraintsPrimary = {upperLegBallSocketConstraint,lKneeHinge,lLegHinge,rigidFeet}
local leftLegConstraintsSecondary = {upperLegBallSocketConstraintAlternative,lKneeBallSocket,lLegBallSocket,rigidFeet}

--Set the constraints of the object
leftLegChain:SetPrimaryConstraints(leftLegConstraintsPrimary)
leftLegChain:SetSecondaryConstraints(leftLegConstraintsSecondary)

--Set the region for the primary constraints
local leftLegRegionPart1 = lowerBody.ConstraintZones.LeftLegPart1
local leftLegRegionPart2 = lowerBody.ConstraintZones.LeftLegPart2
local leftLegRegion = {leftLegRegionPart1,leftLegRegionPart2}
leftLegChain:SetPrimaryConstraintRegion(leftLegRegion)

--Repeat for the right leg-----------

--Same constraints but for right leg
local rightBallSocketConstraintPart = lowerBody.Constraints.rUpperLegConstraint
local rupperLegBallSocketConstraint = BallSocketConstraint.new(rightBallSocketConstraintPart,30,30)

local rKneePart = lowerBody.Constraints.rKneeConstraint
local rKneeBallSocket = BallSocketConstraint.new(rKneePart,20,90)

local rLegPart = lowerBody.Constraints.rLowerLegConstraint
local rLegBallSocket = BallSocketConstraint.new(rLegPart,20,80)

--Make the FABRIK chain not move
local rigidRightFeet = RigidConstraint.new(rightLegChain,4)

--[[
    Create the alternative constraints which uses hinge
    More restrictive and glitchy close to original joint but better fitting and looks nicer visually
]]
local rightUpperLegBallSocketConstraintAlternative = BallSocketConstraint.new(rightBallSocketConstraintPart,40,40)
local rKneeHinge = HingeConstraint.new(rKneePart,90,90)
local rLegHinge = HingeConstraint.new(rLegPart,90,90)

--Construct the constraints table
local rightLegConstraintsPrimary = {rightUpperLegBallSocketConstraintAlternative,rKneeHinge,rLegHinge}
local rightLegConstraintsSecondary = {rupperLegBallSocketConstraint,rKneeBallSocket,rLegBallSocket}

--Set the constraints of the object
rightLegChain:SetPrimaryConstraints(rightLegConstraintsPrimary)
rightLegChain:SetSecondaryConstraints(rightLegConstraintsSecondary)

--Set the region for the primary constraints
local rightLegRegionPart1 = lowerBody.ConstraintZones.RightLegPart1
local rightLegRegionPart2 = lowerBody.ConstraintZones.RightLegPart2
local rightLegRegion = {rightLegRegionPart1,rightLegRegionPart2}
rightLegChain:SetPrimaryConstraintRegion(rightLegRegion)



--turn on debug mode if u want
--leftLegChain:DebugModeOn()


--[[
    Then use the LimbChain object to control the motor every heartbeat
    ]]
RunService.Heartbeat:Connect(function(step)
        
    --The Goal position
    local goalPosition = workspace.MechLTarget.Position
    local goalRightPosition = workspace.MechRTarget.Position

    leftLegChain:IterateOnce(goalPosition,0.1)
    leftLegChain:UpdateMotors()

    rightLegChain:IterateOnce(goalRightPosition,0.1)
    rightLegChain:UpdateMotors()

end)

--Below is testing for Iterate until goal, still pretty glitchy idk why
--Constraints not updating

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
leftLegChain:IterateUntilGoal(workspace.MechLTarget.Position,0.1,15)
leftLegChain:UpdateMotors()
wait(1)
end
]]

