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

--Find a way to include root motor



--spineChain:DebugModeOn()

--Store lower torso c0
local lowerTorsoStore = lowerTorsoMotor.C0
local lowerTorsoStorePosition = lowerTorsoMotor.C0.Position

local xUnit = Vector3.new(1,0,0)
local zUnit = Vector3.new(0,0,1)
local yUnit = Vector3.new(0,1,0)

RunService.Heartbeat:Connect(function()
    --targets for left and right leg
    local target = workspace.LTarget.Position
    local rightTarget = workspace.RTarget.Position

    local dummyHipCF = workspace.Dummy.Body.Hip.CFrame
    --workspace.UpTarget.Position = dummyHipCF.Position + dummyHipCF.UpVector*10
    local upTarget = workspace.UpTarget.Position

    --local newUpVector = workspace.Dummy.Body.UpperTorso.CFrame.Position-dummyHipCF.Position 
  ---  local newRight = -zUnit:Cross(newUpVector)
   -- lowerTorsoMotor.C0 = CFrame.fromMatrix(lowerTorsoStorePosition,newRight,newUpVector)


    leftLegChain:IterateOnce(target,0.1)
    leftLegChain:UpdateMotors()

    spineChain:IterateOnce(upTarget,0.1)
    spineChain:UpdateMotors()

    rightLegChain:IterateOnce(rightTarget,0.1)
    rightLegChain:UpdateMotors()

end)