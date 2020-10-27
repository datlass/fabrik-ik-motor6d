-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

local FabrikConstraint = Object.new("FabrikConstraint")

-- Creates a constraint with axis that depend on the part
function FabrikConstraint.new(Part)
    local obj = FabrikConstraint:make()

    --If there is a part, relative constraint axis is set accordingly
    if Part then
    obj.Part = Part
    obj.PartCF = Part.CFrame
    obj.CenterAxis = Part.CFrame.LookVector
    obj.XAxis = Part.CFrame.RightVector
    obj.YAxis = Part.CFrame.UpVector
    end

    obj.DebugOn = true
    obj.DebugInitialized = false

    return obj
end

--[[
    Method for the constraints to inherit if you want to the axis to change
    Currently broken because of the weld constraint changing the model's CFrame also
]]
function FabrikConstraint:RotateCFrameOrientation(goalCFrameRotation) 

    --disable the weld constraint first to prevent it moving
    self.Part:FindFirstChild("WeldConstraint").Enabled = false

    --Change the constraint
    self.Part.CFrame = CFrame.new(self.Part.CFrame.Position)*goalCFrameRotation 

    --Disable the weld Constraint
    self.Part:FindFirstChild("WeldConstraint").Enabled = true

end

--[[
    Methods to set and get the current axis of the part
    fairly activity intensive goes from 1-2% to 4-6% max
    Also problem as it requires the part motor to update 
]]
function FabrikConstraint:UpdateAxis(PreviousLimbAxisCFrame,JointPosition)

    if not PreviousLimbAxisCFrame then
        --old system planning to remove
        self.CenterAxis = self.Part.CFrame.LookVector
        self.XAxis = self.Part.CFrame.RightVector
        self.YAxis = self.Part.CFrame.UpVector
    else --new system
        self.PartCF = self.Part.CFrame
        local newAxisCF = PreviousLimbAxisCFrame:ToObjectSpace(self.PartCF)
        self.CenterAxis = newAxisCF.LookVector
        self.XAxis = newAxisCF.RightVector
        self.YAxis = newAxisCF.UpVector
        --50.08, 131.88, -11.63
    end

    --self:DebugAxis(JointPosition)
end

--function to visualize direction of the axis
function FabrikConstraint:DebugAxis(JointPosition)

    if not self.DebugInitialized then
        local LimbAxis = Instance.new("WedgePart")
        LimbAxis.BrickColor = BrickColor.random()
        LimbAxis.Name = "LimbAxis"
        LimbAxis.Anchored = true
        LimbAxis.CanCollide = false
        LimbAxis.Size = Vector3.new(2,2,4)
        self.LimbAxis = LimbAxis
        LimbAxis.Parent = workspace
        self.DebugInitialized = true
    else
        
        self.LimbAxis.CFrame = CFrame.fromMatrix(JointPosition,self.XAxis,self.YAxis,self.CenterAxis)

    end

end

return FabrikConstraint
