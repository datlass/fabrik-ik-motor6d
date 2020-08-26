

--Aim is to be similar to backwards but with a table for more robust models
local function Backwards(originCF, targetPos, limbVecTable,limbLengthTable)
	local store = Vector3.new()

	for i = #limbVecTable, 1,-1 do

		local vecSum = Vector3.new(0,0,0)
		--print("Index: ",i," Vectable: ",limbVectable[i])

		for v = 1, i-1, 1 do
			vecSum = vecSum + limbVecTable[v]
		end
		--print("vec sum: ",vecSum)
		
		--Gets the new direction of the new vector along the chain
		--direction is Target Pos to the next point on the chain
		local pointTo = originCF.Position+vecSum-targetPos-store
	--	print(pointTo)
		--constructs the new vectable
		limbVecTable[i] = pointTo.Unit*limbLengthTable[i]
		store = store + limbVecTable[i]
	end
	return originCF, targetPos, limbVecTable,limbLengthTable
end

--Newer iterative methods for forwards
--this is notworking as intended different from original see script_test
local function Forwards(originCF, targetPos, limbVecTable,limbLengthTable)
	local store = Vector3.new()
	for i = 1, #limbVecTable,1 do
		--initialize empty vector for summing
		local vecSum = Vector3.new(0,0,0)
		--print("Index: ",i," Vectable: ",limbVectable[i])
		
		for v = i+1, #limbVecTable, 1 do
			vecSum = vecSum + limbVecTable[v]
		end
		--print("vec sum: ",vecSum)
		
		--Gets the new direction of the new vector along the chain
		--direction of the new vector is from origin to target
		local pointTo = vecSum+targetPos-originCF.Position-store
		--print(pointTo)
		--constructs the new vectable
		limbVecTable[i] = pointTo.Unit*limbLengthTable[i]
		store = store + limbVecTable[i] 
	end
	return originCF, targetPos, limbVecTable,limbLengthTable
end

--Does the cylinder constraining
--Constraint settings is a table
local function ConicalConstraint(yAxis, centerAxis, limbVector, constraintSettings)
	--ellipse width and height of the constraint
	--
	local height = constraintSettings[2]
	local width = constraintSettings[1]

	--Perform vector resolution on limbvector
	--Represents the center of the 2d plane that will be constructed
	local projCenter = limbVector:Dot(centerAxis) * (1 / centerAxis.Magnitude) * centerAxis.Unit

	--position the current limbvector within the 2d plane as another vector
	local posVector = limbVector - projCenter

	--translate into 2d plane
	--create the xAxis from the yAxis use the left hand rule
	local xAxis = (-yAxis:Cross(centerAxis)).Unit

	--Construct the oval
	--Get the X and Y Coordinates
	local yPoint = yAxis:Dot(posVector) / (yAxis.Magnitude)
	local xPoint = xAxis:Dot(posVector) / (xAxis.Magnitude)

	--Construct the oval constrain formula
	local ovalFormula = (xPoint ^ 2) / (width ^ 2) + (yPoint ^ 2) / (height ^ 2)

	--check if the limbvector point is outside the formula constraint
	if ovalFormula > 1 then
		--Obtain the angle from the xaxis
		local angleToXAxis = math.atan(yPoint, xPoint)

		--Place it on the edge of the oval within the contraints placed
		local newXPoint = width * math.cos(angleToXAxis)
		local newYPoint = height * math.sin(angleToXAxis)

		--now reconstruct the limbvector
		--Now we conver it back to a 3d vector
		local newMagnitude = math.sqrt(newXPoint ^ 2 + newYPoint ^ 2)
		--Gets the new direction of the v2 limb
		local newPosVector = posVector.Unit * newMagnitude
		local newDir = projCenter + newPosVector
		--Constructs the new limbvector in a different direction but same length
		limbVector = newDir.Unit * limbVector.Magnitude
	end

	return limbVector
end

--newer function
local function FabrikAlgo(tolerance, originCF, targetPos, limbVecTable, limbLengthTable)
	--get the magnitude of the leg parts
	local maxLength = 0
	--adds all the magnitudes
	for i = 1, #limbLengthTable, 1 do
		maxLength = maxLength+limbLengthTable[i]
	end
	--Get the distance from hip Joint to the target position
	local targetToJoint = targetPos - originCF.Position
	local targetLength = targetToJoint.Magnitude

	--initialize measure feet to where it should be in the world position
	local vecSum = Vector3.new(0,0,0)
	for i = 1, #limbVecTable, 1 do
		vecSum = vecSum+limbVecTable[i]
	end
	local feetJoint = originCF.Position + vecSum
	local feetToTarget = targetPos - feetJoint
	local distanceTolerate = feetToTarget.Magnitude

	--Target point is too far away from the max length the leg can reach then fully extend
	if targetLength > maxLength then
		for i = 1, #limbVecTable, 1 do
			limbVecTable[i] = targetToJoint.Unit*limbLengthTable[i]
		end

		return limbVecTable
	else
		--target point is "reachable"
		--if Distance is more than tolerance then iterate to move the new vectors closer
		--If not then don't execute the iteration to save FPS
		if distanceTolerate >= tolerance then
		 _,_, limbVecTable,_ = Forwards(Backwards(originCF, targetPos, limbVecTable,limbLengthTable))
		end
		 return limbVecTable
	end
end
----

return FabrikAlgo
