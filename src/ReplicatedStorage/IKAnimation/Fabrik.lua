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

        for v = 1, i - 1, 1 do vectorSum = vectorSum + limbVectorTable[v] end

        -- Gets the new direction of the new vector along the chain
        -- direction is Target Pos to the next point on the chain
        local pointTowards = originCF.Position + vectorSum
        local pointFrom = targetPos + vectorSumFromOrigin
        local pointTo = pointTowards - pointFrom

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

        local pointTowards = targetPos + vectorSum
        local pointFrom = originCF.Position + vectorSumFromOrigin

        -- Gets the new direction of the new vector along the chain
        -- direction of the new vector is from origin to target
        local pointTo = pointTowards - pointFrom

        -- constructs the new vectable
        limbVectorTable[i] = pointTo.Unit * limbLengthTable[i]
        vectorSumFromOrigin = vectorSumFromOrigin + limbVectorTable[i]
    end
    return originCF, targetPos, limbVectorTable, limbLengthTable
end

--[[
    Constraining the forwards operation only makes it weird I dont think it simply does enough
    So constraints are also applied to backwards now
    
]]
local function ConstraintBackwards(originCF, targetPos, limbVectorTable,
                                   limbLengthTable, limbConstraintTable)
    local vectorSumFromOrigin = Vector3.new()

    for i = #limbVectorTable, 1, -1 do
        local vectorSum = Vector3.new(0, 0, 0)

        for v = 1, i - 1, 1 do vectorSum = vectorSum + limbVectorTable[v] end

        local pointTowards = originCF.Position + vectorSum
        local pointFrom = targetPos + vectorSumFromOrigin

        -- Gets the new direction of the new vector along the chain
        -- direction is Target Pos to the next point on the chain
        local pointTo = pointTowards - pointFrom

        -- Gotta reverse the direction first
        --The constraint only works if the direction is opposite
        local newLimbVector = -pointTo.Unit * limbLengthTable[i]

        -- Checks if there is a limb constraint for the current limb in the iteration
        if limbConstraintTable[i] and limbConstraintTable[i] ~= nil then

            local limbLength = limbLengthTable[i]
            -- Start the constraint according to the method
            -- print(limbConstraintTable[i])
            newLimbVector = limbConstraintTable[i]:ConstrainLimbVector(
                                pointTowards, newLimbVector, limbLength)
            -- print("Index: ",i,"Vector: ",newLimbVector)
        end
        -- constructs the new vectable
        --Gotta make it negative though to counteract
        limbVectorTable[i] = -newLimbVector
        vectorSumFromOrigin = vectorSumFromOrigin + limbVectorTable[i]
    end
    return originCF, targetPos, limbVectorTable, limbLengthTable
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
        -- target position is the next joint from origin to target
        for v = i + 1, #limbVectorTable, 1 do
            vectorSum = vectorSum + limbVectorTable[v]
        end

        local nextJointPosition = vectorSum + targetPos
        local jointPosition = originCF.Position + vectorSumFromOrigin
        -- Gets the new direction of the new vector along the chain
        -- direction of the new vector is from origin to target
        local pointTo = nextJointPosition - jointPosition
        -- This time constraint the vector using the conical constraint function
        local newLimbVector = pointTo.Unit * limbLengthTable[i]

        -- Checks if there is a limb constraint for the current limb in the iteration
        if limbConstraintTable[i] and limbConstraintTable[i] ~= nil then

            local limbLength = limbLengthTable[i]
            -- Start the constraint according to the method
            -- print(limbConstraintTable[i])
            newLimbVector = limbConstraintTable[i]:ConstrainLimbVector(
                                jointPosition, newLimbVector, limbLength)
            -- print("Index: ",i,"Vector: ",newLimbVector)
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
                ConstraintBackwards(originCF, targetPos, limbVectorTable,
                                    limbLengthTable, limbConstraintTable)
            _, _, limbVectorTable, _ = ConstraintForwards(originCF, targetPos,
                                                          limbVectorTable,
                                                          limbLengthTable,
                                                          limbConstraintTable)

            return limbVectorTable
        else
            -- Does the normal forwards and backwards iteration
            _, _, limbVectorTable, _ = Forwards(
                                           Backwards(originCF, targetPos,
                                                     limbVectorTable,
                                                     limbLengthTable))

            return limbVectorTable
        end
    else    -- Limb is within tolerance so don't iterate for this condition

        return limbVectorTable
    end
end

return FabrikAlgo
