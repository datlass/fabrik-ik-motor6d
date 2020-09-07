--Get service
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

--Modules required
local IKControllerPointer = ReplicatedStorage.Source.ObjectFolder.LimbChain
local LimbChain = require(IKControllerPointer)


--Left leg chain motors
local dummy = workspace.Dummy

--Get the motors of the left leg chain

RunService.Heartbeat:Connect(function()


end)