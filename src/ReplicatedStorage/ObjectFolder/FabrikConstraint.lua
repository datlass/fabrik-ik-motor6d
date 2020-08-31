-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

local FabrikConstraint = Object.new("FabrikConstraint")

-- Creates a constraint with axis that depend on the part
function FabrikConstraint.new(Part)
    local obj = FabrikConstraint:make()

    obj.Part = Part
    obj.CenterAxis = Part.CFrame.LookVector
    obj.XAxis = Part.CFrame.RightVector
    obj.YAxis = Part.CFrame.UpVector

    return obj
end

--[[
    Method for the constraints to inherit if you want to the axis to change
]]
function FabrikConstraint:RotateCFrame(GoalCFrame) 

    self.Part.CFrame = GoalCFrame 

end

--[[
    Method to set and get the current axis of the part
]]
function FabrikConstraint:UpdateAxis()

    self.CenterAxis = self.Part.CFrame.LookVector
    self.XAxis = self.Part.CFrame.RightVector
    self.YAxis = self.Part.CFrame.UpVector

end

return FabrikConstraint
