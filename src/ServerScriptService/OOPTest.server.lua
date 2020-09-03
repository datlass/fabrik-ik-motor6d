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
--It works time

-------------------Import all the Constraints Types-----------------

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
local leftLegChain = LimbChain.new(motorTable)

--Testing the constraint
local testRigidJoint = lowerBody.Constraints.UpperLegConstraint
local upperLegRigidJoint = RigidConstraint.new(leftLegChain,1)

local kneePart = lowerBody.Constraints.KneeConstraint
local lKneeHinge = HingeConstraint.new(kneePart,10,90)

local lLegPart = lowerBody.Constraints.LowerLegConstraint
local lLegHinge = HingeConstraint.new(lLegPart,90,90)
local limbConstraintTable = {nil,lKneeHinge,lLegHinge}

-- Random Parts to debug position of where joints should be according to the algorithm
local part1 = game.Workspace.test1

--Test

--[[
    Then use the object to control the motor every heartbeat
    ]]
RunService.Heartbeat:Connect(function()
        
    --The Goal position
    local goalPosition = workspace.LTarget.Position

    --upperLegRigidJoint:UpdateAxis()
    --lKneeHinge:UpdateAxis()
    --lLegHinge:UpdateAxis()

    leftLegChain:Iterate(0.1,goalPosition,limbConstraintTable)
    leftLegChain:UpdateMotors()

end)
