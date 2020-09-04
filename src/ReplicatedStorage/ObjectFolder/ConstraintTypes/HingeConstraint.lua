

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

    --Gets the part's current CFrame
    self:UpdateAxis()
    
    --Create a plane that is located on the joint with a surface normal to the rightvector
    local planeOnJoint = MathPlane.new(self.XAxis,jointPosition)

    --Find where the limb should end in world position
    local limbVectorWorld = jointPosition+limbVector

    --Project it into the plane
    local pointOnPlane = planeOnJoint:FindClosestPointOnPlane(limbVectorWorld)

    --Get the new direction vector

    local newDir = pointOnPlane-jointPosition

    --Perform the angle check
    local safetyClamp = math.clamp(newDir.Unit:Dot(self.CenterAxis.Unit),-1,1)
    local angle = math.acos(safetyClamp)

    --Dot product to check directionality of the angle to make it angle of elevation or depression
    local dotScalar = newDir.Unit:Dot(self.YAxis.Unit)
    --Divide it by itself to make it 1
    dotScalar = dotScalar/math.abs(dotScalar)

    --Finally obtain angle of depression or elevation
    local angleAdjust = dotScalar*angle

    --Debug the angle to adjust your own Rig
    --print(math.deg(angleAdjust))
    --Issue: It looks pretty buggy if continuosly iterated
    --Checks for elevation or depression
    if angleAdjust < self.AngleOfDepression then

        --Get the rotation Axis
        local rotationAxis = newDir:Cross(self.CenterAxis)

        --Get Cframe and rotate it to max possible angle
        local refCF = self.Part.CFrame*CFrame.fromAxisAngle(rotationAxis,-self.AngleOfDepression)

        return refCF.LookVector.Unit*limbLength

    elseif angleAdjust>self.AngleOfElevation then
        
        --Get the rotation Axis
        local rotationAxis = newDir:Cross(self.CenterAxis)

        --Get Cframe and rotate it to max possible angle
        local refCF = self.Part.CFrame*CFrame.fromAxisAngle(rotationAxis,self.AngleOfElevation)

        return refCF.LookVector.Unit*limbLength 

    end

    --If neither out of bounds that just create vector in plane as is
    local newLimbVector = newDir.Unit*limbLength

    return newLimbVector

end


return HingeConstraint