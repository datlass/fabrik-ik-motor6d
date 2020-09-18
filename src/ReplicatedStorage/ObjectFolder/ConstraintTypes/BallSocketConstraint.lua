

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

--Require the FabrikConstraint for Inheritance
local FabrikConstraintPointer = script.Parent.Parent.FabrikConstraint
local FabrikConstraint = require(FabrikConstraintPointer)

--Initialize the Self Class
local BallSocketConstraint = Object.newExtends("BallSocketConstraint",FabrikConstraint)

--[[--------------------------------------------------------
    Create the constraint
    Parameters:
]]
function BallSocketConstraint.new(Part,AngleOfWidth,AngleOfHeight)
        
    local obj = BallSocketConstraint:super(Part)
   
    obj.AngleOfHeight = math.rad(AngleOfHeight)
    
    obj.AngleOfWidth = math.rad(AngleOfWidth)
    
	return obj
end

--[[
    Constraints the limbVector like a ball socket joint
]]
function BallSocketConstraint:ConstrainLimbVector(jointPosition,limbVector,limbLength)
    
        --Get the parts current CFrame
        --Big problem as its relative to the part attached to the motor
        self:UpdateAxis()

        --Get the Axis
        local centerAxis = self.CenterAxis.Unit
        local yAxis = self.YAxis.Unit
        local xAxis = self.XAxis.Unit

        -- ellipse width and height of the constraint
        local heightCenterAngle = self.AngleOfHeight
        local widthCenterAngle = self.AngleOfWidth
    
        -- Convert Angles into height and width
        -- Height and width are in terms of radius height from origin
        local height = limbLength * math.sin(heightCenterAngle)
        local width = limbLength * math.sin(widthCenterAngle)
    
        -- Perform vector resolution on limbvector
        -- Represents the center of the 2d plane that will be constructed
        -- Also gets the projection scalar which needs to be clamped or else the conicalConstraint fails
        local projScalar = limbVector:Dot(centerAxis) * (1 / centerAxis.Magnitude)
    
        local isOppositeDirection = false

        --Detects the direction of the projection and adjust bool accordingly
        if projScalar < 0 then 
            isOppositeDirection = true
        end
    
        projScalar = math.abs(projScalar)
    
        local minScalar = limbLength * math.cos(widthCenterAngle)
        projScalar = math.clamp(projScalar, minScalar, limbLength)
    
        -- Always make projection scalar positive so that the projCenter faces the center Axis
        local projCenter = projScalar * centerAxis.Unit
    
        -- position the current limbvector within the 2d plane as another vector
        local posVector = limbVector - projCenter
    
        -- translate into 2d plane
    
        -- Construct the oval
        -- Get the X and Y Coordinates
        local yPoint = yAxis:Dot(posVector) / (yAxis.Magnitude)
        local xPoint = xAxis:Dot(posVector) / (xAxis.Magnitude)
    
        -- Construct the oval constrain formula
        local ovalFormula = (xPoint ^ 2) / (width ^ 2) + (yPoint ^ 2) / (height ^ 2)
    
        -- check if the limbvector point is outside the formula constraint
        -- Also checks for directionality if its in the isOppositeDirection then constraint
        if ovalFormula >= 1 or isOppositeDirection then
            -- Obtain the angle from the xaxis
            local angleToXAxis = math.atan(yPoint, xPoint)
    
            -- Place it on the edge of the oval within the contraints placed
            local newXPoint = width * math.cos(angleToXAxis)
            local newYPoint = height * math.sin(angleToXAxis)
    
            -- now reconstruct the limbVector
            -- Now we convert it back to a 3d vector
            local newMagnitude = math.sqrt(newXPoint ^ 2 + newYPoint ^ 2)
    
            -- Gets the new direction of the v2 limb
            local newPosVector = posVector.Unit * newMagnitude
    
            local newDir = projCenter + newPosVector
            -- Constructs the new limbvector in a different direction but same length
            limbVector = newDir.Unit * limbVector.Magnitude
        end
    
        return limbVector

end


return BallSocketConstraint