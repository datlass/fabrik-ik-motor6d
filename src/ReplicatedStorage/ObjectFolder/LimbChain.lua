-- Get Roblox Services
local ReplicatedStorage = game:GetService("ReplicatedStorage")

-- Import the Fabrik Module
-- Deprecated FabrikSolver is the better version but still here
local fabrikPointer = ReplicatedStorage.Source.IKAnimation:WaitForChild("Fabrik")
local FabrikAlgo = require(fabrikPointer)

-- Import the Fabrik Solver Object
local FabrikSolverPointer = ReplicatedStorage.Source.ObjectFolder:WaitForChild("FabrikSolver")
local FabrikSolver = require(FabrikSolverPointer)

-- Import Rotated Region 3
local RotatedRegion3Pointer = ReplicatedStorage.Source.ObjectFolder:WaitForChild("RotatedRegion3")
local RotatedRegion3 = require(RotatedRegion3Pointer)

-- Initialize Object Class
local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)
local LimbChain = Object.new("LimbChain")

--[[
    Initializes the limb chain object
    Calculates the limbs from joint to joint as a vector
    And also measures the limb's length
]]
function LimbChain.new(Motor6DTable,IncludeFoot,SpineMotor)
    --Does the meta table stuff
    local obj = LimbChain:make()

    --Bool option to enable the foot placement system
    obj.IncludeFoot = IncludeFoot

    --attachments for the foot placement system
    --where the ray casting begins
    obj.FootBottomAttachment = nil
    obj.FootBottomRightAttachment = nil

    --The raycast params for the foot placement system
    obj.FootPlacementRaycastParams = nil

    --Stores the motor6ds used and also the original C0 of the first joint
    obj.Motor6DTable = Motor6DTable
    obj.FirstJointC0 = Motor6DTable[1].C0

    --Store the initial C0
    local Motor6DC0Table = {}
    for i = 1, #Motor6DTable, 1 do
        Motor6DC0Table[#Motor6DC0Table+1] = Motor6DTable[i].C0
    end
    obj.Motor6DC0Table = Motor6DC0Table

    -----initialize LimbVectorTable to store the limb vectors and stores it into the object self variable
    local LimbVectorTable = {}
    local IteratedLimbVectorTable = {}

    for i = 1, #Motor6DTable - 1, 1 do
        --print("motorOne: ", Motor6DTable[i].Name,"motorTwo: ",Motor6DTable[i+1].Name)
        --print(LimbChain:JointOneToTwoVector(Motor6DTable[i], Motor6DTable[i + 1]))
        local currentVectorStore = LimbChain:JointOneToTwoVector(Motor6DTable[i], Motor6DTable[i + 1])

        --[[
            the SpineMotor option, allows the creation of a spine from a hip to uppertorso
            This skips the lower torso which is typically connected to the humanoid root part.

            Has to be done as the default limb rotation method using CFrame.fromAxis()
            doesn't work well with how the root part rotates the ENTIRE body.

            also stores it's original C0 Position
        ]]
        obj.SpineMotor = SpineMotor
        if i==1 and SpineMotor then
            obj.SpineMotorC0 = SpineMotor.C0
            --print("reversing limb vector")
            local test1 = Motor6DTable[i].C0.Position
            local test2 = -Motor6DTable[i+1].C0.Position
            currentVectorStore = (test1+test2)
            --print(currentVectorStore.Magnitude)
        end

        LimbVectorTable[#LimbVectorTable + 1] = currentVectorStore
        IteratedLimbVectorTable[#IteratedLimbVectorTable + 1] = currentVectorStore
    end

    obj.LimbVectorTable = LimbVectorTable
    -- Stores the limb vector table in the iterated version
    obj.IteratedLimbVectorTable = IteratedLimbVectorTable

    -- Finds the length of the limb vectors and puts them in a table for the FABRIK algorithm
    -- Important as it reduces .Magnitude calls therefore reduces Activity
    local LimbLengthTable = {}
    for i = 1, #LimbVectorTable, 1 do
        LimbLengthTable[#LimbLengthTable+1] = LimbVectorTable[i].Magnitude
    end
    
    obj.LimbLengthTable = LimbLengthTable

    --[[
        Creates the constraint table variable with nil.
        I was planning on initiailizing the object with constraints but the constraints methods
        require the LimbChain to be CREATED FIRST in order to do stuff like find the rigid joint vector.
    ]]
    obj.LimbConstraintTable = LimbConstraintTable
    --Once the limb vectors are initialized store them in a FabrikSolver object which does the Fabrik iteration
    local LimbFabrikSolver = FabrikSolver.new(IteratedLimbVectorTable,LimbLengthTable,LimbConstraintTable,obj)
    obj.LimbFabrikSolver = LimbFabrikSolver

    --Creates a empty table to store motor c0 for tweening
    obj.MotorsC0Store = {}

    --adds a bool setting for debug mode
    obj.DebugMode = false

    --Adds a setting for enabling constraint regions
    --By default it is nill
    obj.PrimaryConstraintRegionFromParts = nil
    
    --Store the primary and secondary LimbConstraint settings in a table
    obj.PrimaryLimbConstraintTable = {}
    obj.SecondaryLimbConstraintTable = {}

    return obj
end
--[[
    Function that limb chain has to calculate vector limbs
    Input 2 Motor6d joints
    Returns a vector from motorOne to motorTwo joint
    Always constant based on the c0 and c1 Position of the motors
]]
function LimbChain:JointOneToTwoVector(motorOne, motorTwo)
    -- Check if its a motor6d
    if motorOne:IsA("Motor6D") and motorTwo:IsA("Motor6D") then
        local vecOne = motorOne.C1.Position
        local vecTwo = motorTwo.C0.Position
        local combinedVector = vecTwo - vecOne
        return combinedVector
    else
        return "Error: motor 6ds are not inserted in the function"
    end
end
--[[
    Function that executes 1 iteration of the Fabrik Algorithm towards the target position
    Deprecated method uses old IK module which does some inefficient stuff
    (like continually summing up to find the max length every iteration)
]]
function LimbChain:Iterate(tolerance, targetPosition,limbConstraintTable)

    -- Gets the CFrame of the first joint at world space
    local originJointCF = self.Motor6DTable[1].Part0.CFrame * self.FirstJointC0

    --Performs the iteration on the LimbChain object IteratedLimbVectorTable and rewrites it
    --Recursive function
    self.IteratedLimbVectorTable = FabrikAlgo(tolerance, originJointCF, targetPosition+bottomFootOffset, self.IteratedLimbVectorTable, self.LimbLengthTable,limbConstraintTable)
                                              

end

--[[
    Newer iteration method that uses the Fabrik Object instead of a module
    The object stores its own
]]
function LimbChain:IterateOnce(targetPosition,tolerance)

    --[[
    self.TargetPosition = targetPosition
    ]]
    --Should go towards the ankle position instead
    local offsetVector = Vector3.new()
    if self.IncludeFoot then
        --Gets the last motor
        local footMotor = self.Motor6DTable[#self.Motor6DTable]
        local currentLimbPart = footMotor.Part1

        local footBottomToAnkleVector
        --nill check for the foot bottom attachment if not then targetposition is the center
        -- of the part
        if self.FootBottomAttachment and self.FootBottomAttachment.Position then
            footBottomToAnkleVector = footMotor.C1.Position-self.FootBottomAttachment.Position
        else
            footBottomToAnkleVector = footMotor.C1.Position
        end
        offsetVector = currentLimbPart.CFrame:VectorToWorldSpace(footBottomToAnkleVector)

    end
    
    local targetPosition = targetPosition+offsetVector

    --Does the constraint region check and change constraints if in region
    --If not then default to use the primary constraints
    self:CheckAndChangeConstraintRegions(targetPosition)

    -- Gets the CFrame of the first joint at world space
    --Relative to the part 0 which is the Hip and it works
    --I have to use it to map out constraint axis without relying on welds
    local originJointCF = self.Motor6DTable[1].Part0.CFrame * self.FirstJointC0

    --Does the fabrik iteration once if not in goal
    self.IteratedLimbVectorTable = self.LimbFabrikSolver:IterateOnce(originJointCF,targetPosition, tolerance)
                                              

end

function LimbChain:IterateUntilGoal(targetPosition,tolerance,InputtedMaxBreakCount)

    local offsetVector = Vector3.new()
    if self.IncludeFoot then
        local footMotor = self.Motor6DTable[#self.Motor6DTable]
        local currentLimbPart = footMotor.Part1

        local footBottomToAnkleVector
        --nill check for the foot bottom attachment if not then targetposition is the center
        -- of the part
        if self.FootBottomAttachment and self.FootBottomAttachment.Position then
            footBottomToAnkleVector = footMotor.C1.Position-self.FootBottomAttachment.Position
        else
            footBottomToAnkleVector = footMotor.C1.Position
        end
        offsetVector = currentLimbPart.CFrame:VectorToWorldSpace(footBottomToAnkleVector)

    end

    --change offset the target position to the new one
    local targetPosition = targetPosition+offsetVector

    --Does the constraint region check and change constraints
    --If not then default to use the primary constraints
    self:CheckAndChangeConstraintRegions(targetPosition)

    -- Gets the CFrame of the first joint at world space
    local originJointCF = self.Motor6DTable[1].Part0.CFrame * self.FirstJointC0

    --Does the fabrik iteration until goal
    self.IteratedLimbVectorTable = self.LimbFabrikSolver:IterateUntilGoal(originJointCF,targetPosition, tolerance,InputtedMaxBreakCount)
                                              

end
--[[
    Function that rotates the motors to match the algorithm
    Operates by changing the motor's C0 Position to the goal CFrame
    New parameter dt to control lerp
]]
function LimbChain:UpdateMotors(dt)

    -- Gets the CFrame of the Initial joint at world space
    local initialJointCFrame = self.Motor6DTable[1].Part0.CFrame * self.FirstJointC0

    --Vector sum for the for loop to get the series of position of the next joint based on the algorithm
    local vectorSumFromFirstJoint = Vector3.new()

    local iterateUntil = #self.LimbVectorTable

    --[[
        if there is a spine chain skip the first motor
        First motor will usually control the hip (taken by another chain)
        But what we really want to control is the one connected to the RootPart
        motor w/ part 0 = HumanoidRootPart / RootPart,part 1 = LowerTorso
        ]]
    local skip = 0
    if self.SpineMotor then
        skip = 1
        --Then does the rotation for the spine chain
        local firstMotorWorldPosition = initialJointCFrame.Position
        local secondMotorWorldPosition = initialJointCFrame.Position+self.IteratedLimbVectorTable[1]
        local newUpVector = secondMotorWorldPosition - firstMotorWorldPosition

        --Obtains the CFrame of the part0 limb of the motor6d
        local previousLimbPart = self.SpineMotor.Part0
        local previousLimbCF = previousLimbPart.CFrame 

        local SpineMotorWorldPosition = previousLimbCF*self.SpineMotorC0.Position

        local newRight = previousLimbCF.LookVector:Cross(newUpVector)

        local undoPreviousLimbCF = previousLimbCF:Inverse()
        local rotateSpine = CFrame.fromMatrix(SpineMotorWorldPosition,newRight,newUpVector)
        self.SpineMotor.C0 = undoPreviousLimbCF*rotateSpine
    end

    --Iterates and start rotating the limbs starting from the first joint
    for i = 1+skip, iterateUntil, 1 do

        --Obtains the current limb that is being worked on
        local originalVectorLimb = self.LimbVectorTable[i]
        local currentVectorLimb = self.IteratedLimbVectorTable[i]

        --Obtains the CFrame of the part0 limb of the motor6d
        local previousLimbPart = self.Motor6DTable[i].Part0
        local previousLimbCF = previousLimbPart.CFrame

        -- Obtains the CFrame rotation calculation for CFrame.fromAxis
        local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)
        local dotProductAngle = limbVectorRelativeToOriginal.Unit:Dot(currentVectorLimb.Unit)
        local safetyClamp = math.clamp(dotProductAngle, -1, 1)
        local limbRotationAngle = math.acos(safetyClamp)
        local limbRotationAxis = limbVectorRelativeToOriginal:Cross(currentVectorLimb) -- obtain the rotation axis

        --Checks if the axis exists if cross product returns zero somehow
        if limbRotationAxis~=Vector3.new(0,0,0) then
        
        --Gets the world space of the joint from the iterated limb vectors
        if i ~= 1 then
        vectorSumFromFirstJoint = vectorSumFromFirstJoint + self.IteratedLimbVectorTable[i-1]
        end

        --Gets the position of the current limb joint
        local motorPosition = initialJointCFrame.Position + vectorSumFromFirstJoint

        --Now adding a debug mode----------------------------------------------------------------
        --Puts the created parts according to the motor position
        if self.DebugMode then
            workspace["LimbVector:"..i].Position = motorPosition
        end
        ----------------------------------------------------------------
        --Obtain the CFrame operations needed to rotate the limb to the goal
        local undoPreviousLimbCF = previousLimbCF:Inverse()*CFrame.new(motorPosition)
        local rotateLimbCF =CFrame.fromAxisAngle(limbRotationAxis,limbRotationAngle)*CFrame.fromMatrix(Vector3.new(),previousLimbCF.RightVector,previousLimbCF.UpVector)
        
        local goalCF = undoPreviousLimbCF*rotateLimbCF

        local rotationOnly = goalCF-goalCF.Position

        local currentRotation = self.Motor6DTable[i].C0-self.Motor6DTable[i].C0.Position
        local newRotationCF = currentRotation:lerp(rotationOnly,1/4)
        --Changes the current motor6d through c0
        self.Motor6DTable[i].C0 = CFrame.new(self.Motor6DTable[i].C0.Position)*newRotationCF

        end

    
    end

    --Special for the foot motor system which controls the last motor
    if self.IncludeFoot then

        --the last motor index
        local footMotorIndex = #self.IteratedLimbVectorTable

        --Sums up the limb vectors to get the last motor position
        vectorSumFromFirstJoint = vectorSumFromFirstJoint + self.IteratedLimbVectorTable[footMotorIndex]
    

        --Gets the position of the current limb joint
        local motorPosition = initialJointCFrame.Position + vectorSumFromFirstJoint

        --Now adding a debug mode----------------------------------------------------------------
        --Puts the created parts according to the motor position
        if self.DebugMode then
           workspace["LimbVector:"..footMotorIndex+1].Position = motorPosition
        end

        --Now does controls the motor
        --ony if the attachments are set
        if self.FootBottomAttachment and self.FootBottomRightAttachment then
        self:UpdateFootMotor(motorPosition)
        end

    end

end

function LimbChain:UpdateFootMotor(footMotorPosition)

    local footMotorPosition = footMotorPosition

    --get the foot motor which is the last motor of the limb
    local footMotor = self.Motor6DTable[#self.Motor6DTable]
        
    --Obtains the CFrame of the part0 limb of the motor6d
    local previousLimbPart = self.Motor6DTable[#self.Motor6DTable].Part0
    local previousLimbCF = previousLimbPart.CFrame

    --Obtain variables from self object
    local lengthToFloor = self.LengthToFloor
    if not self.LengthToFloor then
        --Default is 10 units down
        lengthToFloor = 10

    end
    local downDirection = lengthToFloor*Vector3.new(0,-1,0)

    local FootPlacementRaycastParams = self.FootPlacementRaycastParams

    --move it up a bit to give raycast space from floor
    local up = Vector3.new(0,1,0)
    local footBottomPosition = self.FootBottomAttachment.WorldPosition+up
    local footBottomRaycastResult = workspace:Raycast(footBottomPosition,downDirection,FootPlacementRaycastParams)

    local footBottomRightPosition = self.FootBottomRightAttachment.WorldPosition+up
    local footBottomRightRaycastResult = workspace:Raycast(footBottomRightPosition,downDirection,FootPlacementRaycastParams)

    --nill check for if its empty
    if footBottomRaycastResult and footBottomRightRaycastResult then

        --Get the average normal vector from both attachments
        local footNormal = (footBottomRaycastResult.Normal+footBottomRightRaycastResult.Normal)/2
        local footRightVector = footBottomRightRaycastResult.Position - footBottomRaycastResult.Position

        local hipRightVector = self.Motor6DTable[1].Part0.CFrame.RightVector

        --average it with how the hip is facing to keep the foot facing forwards
        local footRightVector = (0.8*footRightVector+0.2*hipRightVector)/2

        local undoPreviousLimbCF = previousLimbCF:Inverse()

        --Foot Motor position is the c0 joint in world terms
        local placeAtFootMotor = CFrame.new(footMotorPosition)
        local rotateToFloor = CFrame.fromMatrix(Vector3.new(),footRightVector,footNormal)

        --The goal rotation and Position of the foot C0 motor
        local goalCF = undoPreviousLimbCF*placeAtFootMotor*rotateToFloor

        local rotationOnly = goalCF-goalCF.Position

        --finds the current rotation of the foot currently without position
        local currentRotation = footMotor.C0-footMotor.C0.Position

        --does the lerp for the rotation from current to the new goal
        local newRotationCF = currentRotation:lerp(rotationOnly,1/4)

        --then updates the foot motor
        footMotor.C0 = CFrame.new(footMotor.C0.Position)*newRotationCF

    else
        --default to facing "Up" towards sky
        local footNormal = Vector3.new(0,1,0)

        local footRightVector = previousLimbCF.RightVector

        local undoPreviousLimbCF = previousLimbCF:Inverse()
        --local empty = 
        local orientToFloor = CFrame.fromMatrix(footMotorPosition,footRightVector,footNormal)

        --then updates the foot motor
        footMotor.C0 = undoPreviousLimbCF*orientToFloor

    end
end

--[[
    Not really updated since there is no testing done on it
]]
function LimbChain:StoreMotorsC0(floorNormal)

    --Initialize empty table to store the motor c0 Cframe data 
    local MotorsC0Store = {}

    -- Gets the CFrame of the Initial joint at world space
    local initialJointCFrame = self.Motor6DTable[1].Part0.CFrame * self.FirstJointC0

    --Vector sum for the for loop to get the series of position of the next joint based on the algorithm
    local vectorSumFromFirstJoint = Vector3.new()

    local iterateUntil = #self.LimbVectorTable

        --[[
        if there is a spine chain skip the first motor
        First motor will usually control the hip (taken by another chain)
        But what we really want to control is the one connected to the RootPart
        motor w/ part 0 = HumanoidRootPart / RootPart,part 1 = LowerTorso
        ]]
        local skip = 0
        if self.SpineMotor then
            skip = 1
            --Then does the rotation for the spine chain
            local firstMotorWorldPosition = initialJointCFrame.Position
            local secondMotorWorldPosition = initialJointCFrame.Position+self.IteratedLimbVectorTable[1]
            local newUpVector = secondMotorWorldPosition - firstMotorWorldPosition
    
            --Obtains the CFrame of the part0 limb of the motor6d
            local previousLimbPart = self.SpineMotor.Part0
            local previousLimbCF = previousLimbPart.CFrame 
    
            local SpineMotorWorldPosition = previousLimbCF*self.SpineMotorC0.Position
    
            local newRight = previousLimbCF.LookVector:Cross(newUpVector)
    
            local undoPreviousLimbCF = previousLimbCF:Inverse()
            local rotateSpine = CFrame.fromMatrix(SpineMotorWorldPosition,newRight,newUpVector)
            MotorsC0Store[#MotorsC0Store + 1] = undoPreviousLimbCF*rotateSpine
        end
    
    --Iterates and start rotating the limbs starting from the first joint
    for i = 1+skip, iterateUntil, 1 do

        --Obtains the current limb that is being worked on
        local originalVectorLimb = self.LimbVectorTable[i]
        local currentVectorLimb = self.IteratedLimbVectorTable[i]

        --Obtains the CFrame of the part0 limb of the motor6d
        local previousLimbPart = self.Motor6DTable[i].Part0
        local previousLimbCF = previousLimbPart.CFrame

        -- Obtains the CFrame rotation calculation for CFrame.fromAxis
        local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)
        local dotProductAngle = limbVectorRelativeToOriginal.Unit:Dot(currentVectorLimb.Unit)
        local safetyClamp = math.clamp(dotProductAngle, -1, 1)
        local limbRotationAngle = math.acos(safetyClamp)
        local limbRotationAxis = limbVectorRelativeToOriginal:Cross(currentVectorLimb) -- obtain the rotation axis

        --Checks if the axis exists if cross product returns zero somehow
        if limbRotationAxis~=Vector3.new(0,0,0) then
        
        --Gets the world space of the joint from the iterated limb vectors
        if i ~= 1 then
        vectorSumFromFirstJoint = vectorSumFromFirstJoint + self.IteratedLimbVectorTable[i-1]
        end

        --Gets the position of the current limb joint
        local motorPosition = initialJointCFrame.Position + vectorSumFromFirstJoint

        --Obtain the CFrame operations needed to rotate the limb to the goal
        local undoPreviousLimbCF = previousLimbCF:Inverse()*CFrame.new(motorPosition)
        local rotateLimbCF =CFrame.fromAxisAngle(limbRotationAxis,limbRotationAngle)*CFrame.fromMatrix(Vector3.new(),previousLimbCF.RightVector,previousLimbCF.UpVector)
        
        --This time instead of changing the current motor C0 stores them and adds them in the table
        MotorsC0Store[#MotorsC0Store + 1] = undoPreviousLimbCF*rotateLimbCF

        end

    
    end

    --Updates the objects self table
    self.MotorsC0Store = MotorsC0Store


end

--Prints  the limb vector and iterated limb vector
--For debugging not needed now
function LimbChain:PrintLimbVectors()

    for i=1,#self.LimbVectorTable,1 do
        --print("Limbvector table i:",i," Vector:",self.LimbVectorTable[i])
    end
    for i=1,#self.IteratedLimbVectorTable,1 do
        print("Iterated self table i:",i," Vector:",self.IteratedLimbVectorTable[i])
    end

end

--Gets the original vector limb direction relative to the part
function LimbChain:GetOriginalLimbDirection(limbVectorNumber)

    local i = limbVectorNumber

    --Obtains the current limb that is being worked on
    local originalVectorLimb = self.LimbVectorTable[i]

    --print(self.Motor6DTable[i].Part0)
    --Obtains the CFrame of the part0 limb of the motor6d
    local previousLimbPart = self.Motor6DTable[i].Part0
    local previousLimbCF = previousLimbPart.CFrame

    -- Obtains the vector relative to the previous part0
    local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)
    
    return limbVectorRelativeToOriginal

end

--Testing for feet
function LimbChain:GetOriginalFeetLimb()

    --Obtains the current limb that is being worked on
    local originalVectorLimb = self.extraLimbVector

    --Obtains the CFrame of the part0 limb of the motor6d
    local previousLimbPart = self.Motor6DTable[#self.Motor6DTable].Part0
    local previousLimbCF = previousLimbPart.CFrame

    -- Obtains the vector relative to the previous part0
    local limbVectorRelativeToOriginal = previousLimbCF:VectorToWorldSpace(originalVectorLimb)

    return limbVectorRelativeToOriginal

end

--[[--------------------------------------------------------
    Sets the current constraints that the FABRIK algorithm will take note of

]]
function LimbChain:SetCurrentConstraints(LimbConstraintTable)

    --Changes the constraint table and the fabrik solvers as well
    self.LimbConstraintTable = LimbConstraintTable
    self.LimbFabrikSolver.LimbConstraintTable = LimbConstraintTable

end

--[[--------------------------------------------------------
    Stores the primary constraints that is fits for the model

]]
function LimbChain:SetPrimaryConstraints(PrimaryLimbConstraintTable)

    --Stores in itself the primary table
    self.PrimaryLimbConstraintTable = PrimaryLimbConstraintTable

end

--[[--------------------------------------------------------
    Stores the secondary constraints intended for the worst case scenario for when
    The primary constraints start glitching out due to algorithm unable to work out
    the constraints.
]]
function LimbChain:SetSecondaryConstraints(SecondaryLimbConstraintTable)

    --Stores in itself the primary table
    self.SecondaryLimbConstraintTable = SecondaryLimbConstraintTable

end

--[[--------------------------------------------------------
    Defines the parts that the primary constraints is only allowed to be
    activated in.
]]
function LimbChain:SetPrimaryConstraintRegion(PrimaryConstraintRegionFromParts)

    self.PrimaryConstraintRegionFromParts = PrimaryConstraintRegionFromParts

end

--[[--------------------------------------------------------
    Defines the parts that the primary constraints is only allowed to be
    activated in
]]
function LimbChain:CheckAndChangeConstraintRegions(targetPosition)

    --
    local PrimaryConstraintRegionFromParts = self.PrimaryConstraintRegionFromParts


    --[[
        Checks if the region is set in the first place
        ]]
    if PrimaryConstraintRegionFromParts then

        --bool by default is false
        local isTargetInsideConstraintRegion = false

        --Iterate through the parts and if at least
        for i=1,#PrimaryConstraintRegionFromParts,1 do
            --Creates a region for each part checks if the target position is inside the parts
            local PartConstraintRegion = RotatedRegion3.FromPart(PrimaryConstraintRegionFromParts[i])

            local check = PartConstraintRegion:CastPoint(targetPosition)

            --Checks if at least one region is true so it sets it to true
            if check == true then
             isTargetInsideConstraintRegion = true
            end

        end

        --Checks the condition
        if isTargetInsideConstraintRegion then
            --Inside region so use Primary constraints region
            self:SetCurrentConstraints(self.PrimaryLimbConstraintTable)
        else
            --Out of region use secondary constraints region
            self:SetCurrentConstraints(self.SecondaryLimbConstraintTable)
        end

    else
        --Use primary constraints only if no region is set
        self:SetCurrentConstraints(self.PrimaryLimbConstraintTable)
    end

end--end of function


function LimbChain:DebugModeOn()

    --Stores all the limb vectors in a table so I don't have to self call everytime
    local limbVectors = self.IteratedLimbVectorTable

    local additionalMotor = 0
    if self.IncludeFoot then
        additionalMotor += 1
    end

    --Creates a part for each limb vector
    for i=1,#limbVectors+additionalMotor,1 do
    local part = Instance.new("Part")
    part.Anchored = true
    part.Name = "LimbVector:"..i
    part.BrickColor = BrickColor.random()
    part.Parent = workspace
    end
    self.DebugMode = true
end

return LimbChain
