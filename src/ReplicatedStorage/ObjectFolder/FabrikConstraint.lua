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
    The method all constraints should inherit
    empty as each constraint has it's own special constraint method
]]
function FabrikConstraint:ConstrainLimbVector()

end


--[[
    Method for the constraints to inherit if you want to the axis to change
]]
function FabrikConstraint:ChangeCFrame(goalCFrame) 

    self.Part.CFrame = goalCFrame 

end

--[[
    Methods to set and get the current axis of the part
    fairly activity intensive goes from 1-2% to 4-6% max
]]
function FabrikConstraint:UpdateAxis()

    self.CenterAxis = self.Part.CFrame.LookVector
    self.XAxis = self.Part.CFrame.RightVector
    self.YAxis = self.Part.CFrame.UpVector

end

function FabrikConstraint:UpdateYAxis()

    self.YAxis = self.Part.CFrame.UpVector

end

function FabrikConstraint:UpdateXAxis()

    self.XAxis = self.Part.CFrame.RightVector

end

function FabrikConstraint:UpdateCenterAxis()

    self.CenterAxis = self.Part.CFrame.LookVector

end


return FabrikConstraint
