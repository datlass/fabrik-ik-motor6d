--Get service
local ReplicatedStorage = game:GetService("ReplicatedStorage")

--IK Module
local fabrik = ReplicatedStorage.Source.IKAnimation:WaitForChild("Fabrik")
local FabrikAlgo = require(fabrik)

local IKControllerPointer = ReplicatedStorage.Source.ObjectFolder.limbChain
local limbChain = require(IKControllerPointer)

local leftLegChain = limbChain.new("UpperLeg")

print(leftLegChain.Part)