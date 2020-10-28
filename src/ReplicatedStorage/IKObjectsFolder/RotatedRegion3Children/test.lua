function FabrikSolver:DebugLimbRotationCFrame(index,initialJointPosition,endJointPosition)
    --initizalize first
    if not self.Initialized then
        self.Initialized = true
        local VectorLimbPartTable = {}
        for i, v in pairs(self.LimbVectorTable) do
            local VectorLimbPart = Instance.new("WedgePart")
            VectorLimbPart.BrickColor = BrickColor.random()
            VectorLimbPart.Name = "VectorLimbPart"..i
            VectorLimbPart.Anchored = true
            VectorLimbPart.CanCollide = false
            VectorLimbPart.Size = Vector3.new(0.5,2,self.LimbLengthTable[i])
            VectorLimbPartTable[i] = VectorLimbPart
            VectorLimbPart.Parent = workspace
        end
        self.VectorLimbPartTable=VectorLimbPartTable
    else
        local positionCF = CFrame.new(midPointPosition)
        local rotOnly = limbCFrame-limbCFrame.p
        self.VectorLimbPartTable[index].CFrame = positionCF*rotOnly*CFrame.new(0,0,-self.LimbLengthTable[index]/2)

    end
end