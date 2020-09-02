

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

--Require the FabrikConstraint for Inheritance
local FabrikConstraintPointer = script.Parent.Parent.FabrikConstraint
local FabrikConstraint = require(FabrikConstraintPointer)

--Initialize the Self Class
local RigidConstraint = Object.newExtends("RigidConstraint",FabrikConstraint)

--[[--------------------------------------------------------
    Create the constraint
    Parameters:
]]
function RigidConstraint.new(PartOrLimbChain,LimbNumber)
    local obj

    --[[
        Detects if parameter 1 is a limb chain object, if it is then it constraints to the original limb position
        
    ]]
    if  not PartOrLimbChain:isA("LimbChain") then
    
    obj = RigidConstraint:super(PartOrLimbChain)

    else
        
    obj = RigidConstraint:super()
   
    obj.LimbChain = PartOrLimbChain
    obj.LimbNumber = LimbNumber

    end
    
	return obj
end

--[[
    Constraints the limbVector like a rigid joint
    It doesn't move and the joint points in the part's current direction
    or it points in the motor6ds original direction
    returns a new limbvector vector 3 at full length
]]
function RigidConstraint:ConstrainLimbVector(jointPosition,limbVector,limbLength)

    --Checks if there is a part to set the constraint axis to
    if self.Part ~=nil then

        return self.CenterAxis.Unit*limbLength

    else
        
        return self.LimbChain:GetOriginalLimbDirection(self.LimbNumber).Unit*limbLength

    end
end


return RigidConstraint