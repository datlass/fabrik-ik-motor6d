

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

--Inheritance
local FabrikConstraintPointer = script.Parent.Parent.FabrikConstraint
local FabrikConstraint = require(FabrikConstraintPointer)

local HingeConstraint = Object.newExtends("HingeConstraint",FabrikConstraint)

function HingeConstraint.new(Part,AngleOfElevation,AngleOfDepression)
	local obj = HingeConstraint:super(Part)
    
    obj.AngleOfElevation = AngleOfElevation
    
    obj.AngleOfDepression = AngleOfDepression

	return obj
end

return HingeConstraint