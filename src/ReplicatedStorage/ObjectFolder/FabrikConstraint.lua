-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

local FabrikConstraint = Object.new("FabrikConstraint")
--local FabrikConstraint = Object.newExtends("FabrikConstraint",?)

--Creates a constraint with axis that depend on the part
function FabrikConstraint.new(Part)
	local obj = FabrikConstraint:make()
    --local obj = FabrikConstraint:super()
    obj.Part = Part
    obj.CenterAxis = Part.LookVector
    obj.XAxis = Part.RightVector
    obj.YAxis = Part.UpVector

	return obj
end

--[[
    Method for the constraints to inherit if you want to the axis to change
]]
function FabrikConstraint:RotateCFrame(GoalCFrame)
    self.Part.CFrame = GoalCFrame
end

return FabrikConstraint