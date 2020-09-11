-- File to just shift alt F format all

function LimbChain:UpdateMotorAtIndex(Index)

    -- Gets the CFrame of the Initial joint at world space
    local initialJointCFrame = self.Motor6DTable[1].Part0.CFrame *
                                   self.FirstJointC0

    -- Vector sum for the for loop to get the series of position of the next joint based on the algorithm
    local vectorSumFromFirstJoint = Vector3.new()

    -- Only rotates at motor at index

    -- Obtains the current limb that is being worked on
    local originalVectorLimb = self.LimbVectorTable[i]
    local currentVectorLimb = self.IteratedLimbVectorTable[i]

    -- Obtains the CFrame of the part0 limb of the motor6d
    local previousLimbPart = self.Motor6DTable[i].Part0
    local previousLimbCF = previousLimbPart.CFrame

    -- Obtains the CFrame rotation calculation for CFrame.fromAxis
    local limbVectorRelativeToOriginal =
        previousLimbCF:VectorToWorldSpace(originalVectorLimb)
    local dotProductAngle = limbVectorRelativeToOriginal.Unit:Dot(
                                currentVectorLimb.Unit)
    local safetyClamp = math.clamp(dotProductAngle, -1, 1)
    local limbRotationAngle = math.acos(safetyClamp)
    local limbRotationAxis = limbVectorRelativeToOriginal:Cross(
                                 currentVectorLimb) -- obtain the rotation axis

    -- Checks if the axis exists if cross product returns zero somehow
    if limbRotationAxis ~= Vector3.new(0, 0, 0) then

        -- Gets the world space of the joint from the iterated limb vectors
        if i ~= 1 then
            vectorSumFromFirstJoint = vectorSumFromFirstJoint +
                                          self.IteratedLimbVectorTable[i - 1]
        end

        -- Gets the position of the current limb joint
        local motorPosition = initialJointCFrame.Position +
                                  vectorSumFromFirstJoint

        -- Now adding a debug mode----------------------------------------------------------------
        -- Puts the created parts according to the motor position
        if self.DebugMode then
            workspace["LimbVector:" .. i].Position = motorPosition
        end
        ----------------------------------------------------------------
        -- Obtain the CFrame operations needed to rotate the limb to the goal
        local undoPreviousLimbCF = previousLimbCF:Inverse() *
                                       CFrame.new(motorPosition)
        local rotateLimbCF = CFrame.fromAxisAngle(limbRotationAxis,
                                                  limbRotationAngle) *
                                 CFrame.fromMatrix(Vector3.new(),
                                                   previousLimbCF.RightVector,
                                                   previousLimbCF.UpVector)

        -- Changes the current motor6d through c0
        self.Motor6DTable[i].C0 = undoPreviousLimbCF * rotateLimbCF

    end

end
