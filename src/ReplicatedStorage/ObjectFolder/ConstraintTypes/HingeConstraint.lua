

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

--Require the FabrikConstraint for Inheritance
local FabrikConstraintPointer = script.Parent.Parent.FabrikConstraint
local FabrikConstraint = require(FabrikConstraintPointer)

--Initialize the Self Class
local HingeConstraint = Object.newExtends("HingeConstraint",FabrikConstraint)

--Require the MathPlane object to the math with equation of planes
local MathPlanePointer = script.Parent.Parent.MathPlane
local MathPlane = require(MathPlanePointer)

function HingeConstraint.new(Part,AngleOfElevation,AngleOfDepression)
	local obj = HingeConstraint:super(Part)
    
    obj.AngleOfElevation = AngleOfElevation
    
    obj.AngleOfDepression = AngleOfDepression

	return obj
end



return HingeConstraint