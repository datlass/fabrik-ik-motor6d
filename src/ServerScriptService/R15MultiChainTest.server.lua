--This script controls the r15 avatar

--Get service
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

--Modules required
local IKControllerPointer = ReplicatedStorage.Source.ObjectFolder.LimbChain
local LimbChain = require(IKControllerPointer)


--Point to dummy in the workspace
local dummy = workspace.R15Dummy

------------Left Leg--------------------------

--Get the motors of the left leg chain
local rootMotor = dummy.LowerTorso.Root
local lUpperLegMotor = dummy.LeftUpperLeg.LeftHip
local lLowerLegMotor = dummy.LeftLowerLeg.LeftKnee
local lfoot = dummy.LeftFoot.LeftAnkle

--Create the left leg chain
local leftLegMotorTable = {lUpperLegMotor,lLowerLegMotor,lfoot}
local leftLegChain = LimbChain.new(leftLegMotorTable)

------------Right Leg--------------------------

--Get the motors of the right leg chain
local rUpperLegMotor = dummy.RightUpperLeg.RightHip
local rLowerLegMotor = dummy.RightLowerLeg.RightKnee
local rfoot = dummy.RightFoot.RightAnkle

--Create the right leg chain
local rightLegMotorTable = {rUpperLegMotor,rLowerLegMotor,rfoot}
local rightLegChain = LimbChain.new(rightLegMotorTable)

----------Spine Chain--------------------
--idk waist
local waistMotor = dummy.UpperTorso.Waist
local neck = dummy.Head.Neck

--create the spine chain
local spineMotorTable = {rootMotor,neck}
local spineChain = LimbChain.new(spineMotorTable,false,waistMotor)



RunService.Heartbeat:Connect(function()
    --targets for left and right leg and spine
    local target = workspace.LTarget.Position
    local rightTarget = workspace.RTarget.Position
    local upTarget = workspace.R15UpTarget.Position

    leftLegChain:IterateOnce(target,0.1)
    leftLegChain:UpdateMotors()

    spineChain:IterateOnce(upTarget,0.1)
    spineChain:UpdateMotors()

    rightLegChain:IterateOnce(rightTarget,0.1)
    rightLegChain:UpdateMotors()

end)
