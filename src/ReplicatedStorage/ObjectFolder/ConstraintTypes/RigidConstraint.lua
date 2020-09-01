

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

--Require the FabrikConstraint for Inheritance
local FabrikConstraintPointer = script.Parent.Parent.FabrikConstraint
local FabrikConstraint = require(FabrikConstraintPointer)

--Initialize the Self Class
local RigidConstraint = Object.newExtends("RigidConstraint",FabrikConstraint)

--Require the MathPlane object to the math with equation of planes
local MathPlanePointer = script.Parent.Parent.MathPlane
local MathPlane = require(MathPlanePointer)

function RigidConstraint.new(Part)
	local obj = RigidConstraint:super(Part)
    
	return obj
end

--[[
    Constraints the limbVector like a rigid joint
    It doesn't move
    returns a new limbvector vector 3 at full length
]]
function RigidConstraint:ConstrainLimbVector(jointPosition,limbVector,limbLength)

    return self.CenterAxis.Unit*limbLength

end


return RigidConstraint