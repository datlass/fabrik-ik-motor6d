--Example script for controlling the left leg of a model in the workspace

--Get service
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

--Modules required
local IKControllerPointer = ReplicatedStorage.Source.LimbChain
local LimbChain = require(IKControllerPointer)

-------------------Import all the Constraints Types-----------------

--BallSocketConstraint
local BallSocketConstraintPointer = ReplicatedStorage.Source.LimbChain.FabrikConstraint.BallSocketConstraint
local BallSocketConstraint = require(BallSocketConstraintPointer)

--HingeConstraint
local HingeConstraintPointer = ReplicatedStorage.Source.LimbChain.FabrikConstraint.HingeConstraint
local HingeConstraint = require(HingeConstraintPointer)

--Rigid Constraint
local RigidConstraintPointer = ReplicatedStorage.Source.LimbChain.FabrikConstraint.RigidConstraint
local RigidConstraint = require(RigidConstraintPointer)

----------------------------------------------------------------

-- Pointers
local lowerBody = workspace.LowerBody
--Name of the Motor6D's in order from hip to foot
local leftLegMotors = {"LUpperLeg","LKnee","LLowerLeg","LFeet"}
local rightLegMotors = {"RUpperLeg","RKnee","RLowerLeg","RFeet"}

--Finds the instances for the Limb Chain to be initialized
--converts the names within the tables to motor6d instances within the model
LimbChain.convertNamesToMotor6DInstancesInModel(lowerBody,leftLegMotors,rightLegMotors)

--Initialize the left leg chain
local leftLegChain = LimbChain.new(leftLegMotors,true)

--Foot placement system
local footParams = RaycastParams.new()
local ignoreParts = workspace:WaitForChild("RayFilterFolder")
footParams.FilterDescendantsInstances = {lowerBody,ignoreParts}

leftLegChain.FootPlacementRaycastParams = footParams
leftLegChain.LengthToFloor = 20

--Get all the attachments in the left foot
leftLegChain.FootBottomAttachment = lowerBody.LeftLeg.LFeet.FootBottom
leftLegChain.FootBottomRightAttachment = lowerBody.LeftLeg.LFeet.FootBottomRight
--Initialize the right leg chain
local rightLegChain = LimbChain.new(rightLegMotors,true)

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
local lKneeHinge = HingeConstraint.new(kneePart,80,80)
local lLegHinge = HingeConstraint.new(lLegPart,100,100)

--Set up two constraint tables to allow
local leftLegConstraintsPrimary = {upperLegBallSocketConstraint,lKneeHinge,lLegHinge}
local leftLegConstraintsSecondary = {upperLegBallSocketConstraintAlternative,lKneeBallSocket,lLegBallSocket}

--Set the constraints of the object
leftLegChain.PrimaryLimbConstraintTable = leftLegConstraintsPrimary
leftLegChain.SecondaryLimbConstraintTable = leftLegConstraintsSecondary

--Set the region for the primary constraints
local leftLegRegionPart1 = lowerBody.ConstraintZones.LeftLegPart1
local leftLegRegionPart2 = lowerBody.ConstraintZones.LeftLegPart2
local leftLegRegion = {leftLegRegionPart1,leftLegRegionPart2}

leftLegChain.PrimaryConstraintRegionFromParts = leftLegRegion

--Repeat for the right leg-----------

--Same constraints but for right leg
local rightBallSocketConstraintPart = lowerBody.Constraints.rUpperLegConstraint
local rupperLegBallSocketConstraint = BallSocketConstraint.new(rightBallSocketConstraintPart,30,30)

local rKneePart = lowerBody.Constraints.rKneeConstraint
local rKneeBallSocket = BallSocketConstraint.new(rKneePart,20,90)

local rLegPart = lowerBody.Constraints.rLowerLegConstraint
local rLegBallSocket = BallSocketConstraint.new(rLegPart,20,80)

--[[
    Create the alternative constraints which uses hinge
    More restrictive and glitchy close to original joint but better fitting and looks nicer visually
]]
local rightUpperLegBallSocketConstraintAlternative = BallSocketConstraint.new(rightBallSocketConstraintPart,40,40)
local rKneeHinge = HingeConstraint.new(rKneePart,80,80)
local rLegHinge = HingeConstraint.new(rLegPart,100,100)

--Construct the constraints table
local rightLegConstraintsPrimary = {rightUpperLegBallSocketConstraintAlternative,rKneeHinge,rLegHinge}
local rightLegConstraintsSecondary = {rupperLegBallSocketConstraint,rKneeBallSocket,rLegBallSocket}

--Set the constraints of the object
rightLegChain.PrimaryLimbConstraintTable = rightLegConstraintsPrimary
rightLegChain.SecondaryLimbConstraintTable = rightLegConstraintsSecondary

--Set the region for the primary constraints
local rightLegRegionPart1 = lowerBody.ConstraintZones.RightLegPart1
local rightLegRegionPart2 = lowerBody.ConstraintZones.RightLegPart2
local rightLegRegion = {rightLegRegionPart1,rightLegRegionPart2}
rightLegChain.PrimaryConstraintRegionFromParts = rightLegRegion

--turn on debug mode if u want
--leftLegChain:DebugModeOn(true,true,false)
--rightLegChain:DebugModeOn(false,true,false)

local down = Vector3.new(0,-20,0)
--[[
    Then use the LimbChain object to control the motor every heartbeat
    ]]
rightLegChain:InitDragDebug()

RunService.Heartbeat:Connect(function(step)
        
    --The Goal position
    local goalPosition = workspace.MechLTarget.Position
    local goalRightPosition = workspace.MechRTarget.Position
    local rayResult = workspace:Raycast(goalPosition,down,footParams)
    if rayResult then
       leftLegChain:IterateOnce(rayResult.Position,0.1)
    end
    leftLegChain:UpdateMotors()

    --InitDragDebug does the samething
    --rightLegChain:IterateOnce(goalRightPosition,0.1)
    --rightLegChain:UpdateMotors()

end)

--Below is testing for Iterate until goal, works I guess

--[[
    
for i=1,10000,1 do
leftLegChain:IterateUntilGoal(workspace.MechLTarget.Position,0.1,15)
leftLegChain:UpdateMotors()
wait(1)
end
]]

