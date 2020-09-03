          
          --Old constraint forwards code

          -- The axis of constraint is relative to the initial joint placement
            local yAxis = limbConstraintTable[i][1]
            local centerAxis = limbConstraintTable[i][2]
            local angles = limbConstraintTable[i][3]
            pointTo = ConicalConstraint(pointTo, limbLengthTable[i], yAxis,
                                        centerAxis, angles)


--Old Conical constraint, I imported into an object with the same functionality

--[[
	Given a vector of where the limb should be this function constraints it within a spherical cone
	Returns a new limb vector in the constrained position
	Constraint Settings are (width, height)
	--notes can get laggy with high activity 3%-10% find ways to optimize later
]]
local function ConicalConstraint(limbVector, limbVectorLength, yAxis,
  centerAxis, constraintSettings)
-- ellipse width and height of the constraint
local heightCenterAngle = math.rad(constraintSettings[2])
local widthCenterAngle = math.rad(constraintSettings[1])

-- Convert Angles into height and width
-- Height and width are in terms of radius height from origin
local height = limbVectorLength * math.sin(heightCenterAngle)
local width = limbVectorLength * math.sin(widthCenterAngle)

-- Perform vector resolution on limbvector
-- Represents the center of the 2d plane that will be constructed
-- Also gets the projection scalar which needs to be clamped or else the conicalConstraint fails
local projScalar = limbVector:Dot(centerAxis) * (1 / centerAxis.Magnitude)

local isOppositeDirection = false
if projScalar < 0 then
isOppositeDirection = true
end

projScalar = math.abs(projScalar)

local minScalar = limbVectorLength * math.cos(widthCenterAngle)
projScalar = math.clamp(projScalar, minScalar, limbVectorLength)

-- Always make projection scalar positive so that the projCenter faces the center Axis
local projCenter = projScalar * centerAxis.Unit

-- position the current limbvector within the 2d plane as another vector
local posVector = limbVector - projCenter

-- translate into 2d plane
-- create the xAxis from the yAxis use the left hand rule
local xAxis = (-yAxis:Cross(centerAxis)).Unit

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