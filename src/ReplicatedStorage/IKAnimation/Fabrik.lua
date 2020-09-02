--[[
	Iterate the backward chain of the FABRIK Algorithm
	Function should be called before the Forwards function in order to prevent the vector direction from changing
	Assumes vector chain is from startpoint to endpoint
	Returns parameters with new vector chain direction from endpoint to startpoint
]] local function Backwards(originCF, targetPos, limbVectorTable,
                            limbLengthTable)
    local vectorSumFromOrigin = Vector3.new()

    for i = #limbVectorTable, 1, -1 do
        local vectorSum = Vector3.new(0, 0, 0)
        -- print("Index: ",i," Vectable: ",limbVectorTable[i])

        for v = 1, i - 1, 1 do vectorSum = vectorSum + limbVectorTable[v] end
        -- print("vec sum: ",vectorSum)

        -- Gets the new direction of the new vector along the chain
        -- direction is Target Pos to the next point on the chain
        local pointTo = originCF.Position + vectorSum - targetPos -
                            vectorSumFromOrigin
        --	print(pointTo)
        -- constructs the new vectable
        limbVectorTable[i] = pointTo.Unit * limbLengthTable[i]
        vectorSumFromOrigin = vectorSumFromOrigin + limbVectorTable[i]
    end
    return originCF, targetPos, limbVectorTable, limbLengthTable
end

--[[
	Does the forward chain of the FABRIK Algorithm
	Function should be called after the Backwards function in order to prevent the vector direction from changing
	Assumes vector chain is from endpoint to startpoint
	Returns parameters with new vector chain direction from Startpoint to EndPoint
]]
local function Forwards(originCF, targetPos, limbVectorTable, limbLengthTable)
    local vectorSumFromOrigin = Vector3.new()
    for i = 1, #limbVectorTable, 1 do
        -- initialize empty vector for summing
        local vectorSum = Vector3.new(0, 0, 0)
        -- print("Index: ",i," Vectable: ",limbVectorTable[i])

        for v = i + 1, #limbVectorTable, 1 do
            vectorSum = vectorSum + limbVectorTable[v]
        end
        -- print("vec sum: ",vectorSum)

        -- Gets the new direction of the new vector along the chain
        -- direction of the new vector is from origin to target
        local pointTo = vectorSum + targetPos - originCF.Position -
                            vectorSumFromOrigin
        -- print(pointTo)
        -- constructs the new vectable
        limbVectorTable[i] = pointTo.Unit * limbLengthTable[i]
        vectorSumFromOrigin = vectorSumFromOrigin + limbVectorTable[i]
    end
    return originCF, targetPos, limbVectorTable, limbLengthTable
end

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
    if projScalar < 0 then isOppositeDirection = true end

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

--[[
    A custom constraint method to totally prevent rotation by using planes
    lets see how this goes
]]
local function HingeConstraint(limbVector, limbVectorLength, yAxis,
    centerAxis, constraintSettings)

end
--[[
	Same as forwards Function
	limbConstraintTable

]]
local function ConstraintForwards(originCF, targetPos, limbVectorTable,
                                  limbLengthTable, limbConstraintTable)
    local vectorSumFromOrigin = Vector3.new()
    for i = 1, #limbVectorTable, 1 do
        -- initialize empty vector for summing
        local vectorSum = Vector3.new(0, 0, 0)

        -- Sums up the vectors in order to get the target position on the chain
        --target position is the next joint from origin to target
        for v = i + 1, #limbVectorTable, 1 do
            vectorSum = vectorSum + limbVectorTable[v]
        end

        local nextJointPosition =vectorSum + targetPos
        local jointPosition = originCF.Position + vectorSumFromOrigin
        -- Gets the new direction of the new vector along the chain
        -- direction of the new vector is from origin to target
        local pointTo = nextJointPosition - jointPosition
        -- This time constraint the vector using the conical constraint function
        local newLimbVector = pointTo.Unit * limbLengthTable[i]
        
        -- Checks if there is a limb constraint for the current limb in the iteration
        if limbConstraintTable[i] and limbConstraintTable[i] ~= nil then
           
            local limbLength = limbLengthTable[i]
            --Start the constraint according to the method
            --print(limbConstraintTable[i])
            newLimbVector = limbConstraintTable[i]:ConstrainLimbVector(jointPosition,newLimbVector,limbLength)
            --print("Index: ",i,"Vector: ",newLimbVector)
        end
        -- constructs the new vectable
        limbVectorTable[i] = newLimbVector
        vectorSumFromOrigin = vectorSumFromOrigin + limbVectorTable[i]
    end
    return originCF, targetPos, limbVectorTable, limbLengthTable
end

-- newer function
local function FabrikAlgo(tolerance, originCF, targetPos, limbVectorTable,
                          limbLengthTable, limbConstraintTable)
    -- get the magnitude of the leg parts
    local maxLength = 0
    -- adds all the magnitudes
    for i = 1, #limbLengthTable, 1 do
        maxLength = maxLength + limbLengthTable[i]
    end

    -- initialize measure feet to where it should be in the world position
    local vectorSum = Vector3.new(0, 0, 0)
    for i = 1, #limbVectorTable, 1 do
        vectorSum = vectorSum + limbVectorTable[i]
    end
    local feetJoint = originCF.Position + vectorSum
    local feetToTarget = targetPos - feetJoint
    local distanceTolerate = feetToTarget.Magnitude

    -- target point is "reachable"
    -- if Distance is more than tolerance then iterate to move the new vectors closer
    -- If not then don't execute the iteration to save FPS

    if distanceTolerate >= tolerance then
        -- If there is a constraint table then use constraint forwards else use unconstraint
        if limbConstraintTable ~= nil then
            local originCF, targetPos, limbVectorTable, limbLengthTable =
                Backwards(originCF, targetPos, limbVectorTable, limbLengthTable)
            _, _, limbVectorTable, _ = ConstraintForwards(originCF, targetPos,
                                                          limbVectorTable,
                                                          limbLengthTable,
                                                          limbConstraintTable)

            return limbVectorTable
        else
            _, _, limbVectorTable, _ = Forwards(
                                           Backwards(originCF, targetPos,
                                                     limbVectorTable,
                                                     limbLengthTable))

            return limbVectorTable
        end
    else
        return limbVectorTable
    end
end

return FabrikAlgo
