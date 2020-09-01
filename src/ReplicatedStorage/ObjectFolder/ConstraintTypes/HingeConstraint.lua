

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
    
    obj.AngleOfElevation = math.rad(AngleOfElevation)
    
    obj.AngleOfDepression = -math.rad(AngleOfDepression)

	return obj
end

--[[
    Constraints the limbVector like a hinge
    returns a new limbvector
]]
function HingeConstraint:ConstrainLimbVector(jointPosition,limbVector,limbLength)
    --print("Constraining")
    --Create a plane that is located on the joint with a surface normal to the rightvector
    local planeOnJoint = MathPlane.new(self.XAxis,jointPosition)

    --Find where the limb should end in world position
    local limbVectorWorld = jointPosition+limbVector

    --Project it into the plane
    local pointOnPlane = planeOnJoint:FindClosestPointOnPlane(limbVectorWorld)

    --Get the new direction vector

    local newDir = pointOnPlane-jointPosition


    local newLimbVector = newDir.Unit*limbLength

    return newLimbVector

end


return HingeConstraint