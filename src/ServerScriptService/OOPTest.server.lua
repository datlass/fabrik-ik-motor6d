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

--HingeConstraint
local HingeConstraintPointer = ReplicatedStorage.Source.ObjectFolder.ConstraintTypes.HingeConstraint
local HingeConstraint = require(HingeConstraintPointer)

--Rigid Constraint
local RigidConstraintPointer = ReplicatedStorage.Source.ObjectFolder.ConstraintTypes.RigidConstraint
local RigidConstraint = require(RigidConstraintPointer)

--Testing the constraint
local part = workspace.UpperLegConstraint
local upperLegRigidJoint = RigidConstraint.new(part)

local kneePart = workspace.KneeConstraint
local lKneeHinge = HingeConstraint.new(kneePart,30,30)


--[[
--Test the mathplane object
local MathPlanePointer = ReplicatedStorage.Source.ObjectFolder.MathPlane
local MathPlane = require(MathPlanePointer)

local plane = MathPlane.new(Vector3.new(3,4,1),Vector3.new(-1,1,0))
--It works?? but floating point error
local point = plane:FindClosestPointOnPlane(Vector3.new(1,0,1))
local bool = plane:IsPointOnPlane(Vector3.new(-1,1,0))
print(point,bool)
]]

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

--[[
    Then use the object to control the motor every heartbeat
    ]]
RunService.Heartbeat:Connect(function()
        
    --The Goal position
    local goalPosition = workspace.LTarget.Position

    upperLegRigidJoint:UpdateAxis()
    lKneeHinge:UpdateAxis()
    local limbConstraintTable = {upperLegRigidJoint,lKneeHinge}

    leftLegChain:Iterate(0.1,goalPosition,limbConstraintTable)
    leftLegChain:UpdateMotors()

end)
