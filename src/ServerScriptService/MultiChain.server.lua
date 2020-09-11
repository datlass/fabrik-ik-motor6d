--This controls Mr.Flop Man

--Get service
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

--Modules required
local IKControllerPointer = ReplicatedStorage.Source.ObjectFolder.LimbChain
local LimbChain = require(IKControllerPointer)


--Left leg chain motors
local dummy = workspace.Dummy

--Now including torso motors and head
local upperTorsoMotor = dummy.Body.LowerTorso.UpperTorso
local headMotor = dummy.Body.UpperTorso.Head

--Dont mess with root motor or else entire body will speeen
local lowerTorsoMotor = dummy.Body.HumanoidRootPart.LowerTorso

--Get the motors of the left leg chain
local hipMotor = dummy.Body.LowerTorso.Hip
local lUpperLegMotor = dummy.Body.Hip.LUpperLeg
local lLowerLegMotor = dummy.LLeg.LUpperLeg.LLowerLeg
local lfoot = dummy.LLeg.LLowerLeg.LFeet

--Get the motors of the right leg chain
local rUpperLegMotor = dummy.Body.Hip.RUpperLeg
local rLowerLegMotor = dummy.RLeg.RUpperLeg.RLowerLeg
local rfoot = dummy.RLeg.RLowerLeg.RFeet

--Create the left leg chain
local leftLegMotorTable = {hipMotor,lUpperLegMotor,lLowerLegMotor,lfoot}
local leftLegChain = LimbChain.new(leftLegMotorTable)

--create the right leg chain
local rightLegMotorTable = {rUpperLegMotor,rLowerLegMotor,rfoot}
local rightLegChain = LimbChain.new(rightLegMotorTable)

--create the spine chain
local spineMotorTable = {hipMotor,upperTorsoMotor,headMotor}
local spineChain = LimbChain.new(spineMotorTable,false,lowerTorsoMotor)

RunService.Heartbeat:Connect(function()
    --targets for left and right leg and spine
    --Oops I think I swapped the perspective of the left and right of the feet, oh well
    local target = workspace.BigLTarget.Position
    local rightTarget = workspace.BigRTarget.Position
    local upTarget = workspace.UpTarget.Position

    leftLegChain:IterateOnce(target,0.1)
    leftLegChain:UpdateMotors()

    spineChain:IterateOnce(upTarget,0.1)
    spineChain:UpdateMotors()

    rightLegChain:IterateOnce(rightTarget,0.1)
    rightLegChain:UpdateMotors()

end)