

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

--[[
    Constraints the limbVector like a hinge
    returns a new limbvector
]]
function HingeConstraint:ConstraintLimbVector(jointPosition,limbVector,limbLength)

    --Create a plane that is located on the joint with a surface normal to the rightvector
    local planeOnJoint = MathPlane.new(self.RightVector,jointPosition)

    local limbVectorEndPosition = jointPosition+limbVector

    --Measure the angle of elevation or depression within the constraints
    --Always the absolute angle between them
    local angle = limbVector.Unit:Dot(self.CenterAxis.Unit)

    --Measure the directionality of the angle relative to the y axis via dot product
    --If this value is negative then they are facing in the opposite direction
    local yAxisScalar = limbVector:Dot(self.YAxis)

    --If negative then make the angle value negative so its angle of depression from
    if yAxisScalar < 0 then
        angle = -angle
    end

    --Checks if the limb vector is inside the plane
    if planeOnJoint:IsPointOnPlane(jointPosition) then
        
        --Checks if the limb vector is inside the angle constraints
        
        return limbVector

    else

        return limbVector

    end
end


return HingeConstraint