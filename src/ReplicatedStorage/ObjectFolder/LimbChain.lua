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
function LimbChain.new(Motor6DTable,IncludeAppendage,SpineMotor)
    --Does the meta table stuff
    local obj = LimbChain:make()

    obj.IncludeAppendage = IncludeAppendage

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

    --Checks if this setting is true then adds the c1 joint into the vector limb
    if IncludeAppendage == true then
        local lastMotor = Motor6DTable[#Motor6DTable]
        --print(lastMotor)
        --local test = -lastMotor.C1.Position
        --This is more accurate than getting C1 position for some reason
        local ExtraLimbVector = lastMotor.Part1.CFrame.Position-(lastMotor.Part0.CFrame*lastMotor.C0.Position)

        LimbVectorTable[#LimbVectorTable + 1] = ExtraLimbVector
        IteratedLimbVectorTable[#IteratedLimbVectorTable + 1] = ExtraLimbVector

        obj.ExtraLimbVector = ExtraLimbVector
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
    self.IteratedLimbVectorTable = FabrikAlgo(tolerance, originJointCF, targetPosition, self.IteratedLimbVectorTable, self.LimbLengthTable,limbConstraintTable)
                                              

end

--[[
    Newer iteration method that uses the Fabrik Object instead of a module
    The object stores its own
]]
function LimbChain:IterateOnce(targetPosition,tolerance)
    
    --Does the constraint region check and change constraints if in region
    --If not then default to use the primary constraints
    self:CheckAndChangeConstraintRegions(targetPosition)

    -- Gets the CFrame of the first joint at world space
    --local originJointCF = self.Motor6DTable[1].Part0.CFrame * self.FirstJointC0
    local originJointCF = self.Motor6DTable[1].Part0.CFrame * self.FirstJointC0

    --Does the fabrik iteration once if not in goal
    self.IteratedLimbVectorTable = self.LimbFabrikSolver:IterateOnce(originJointCF,targetPosition, tolerance)
                                              

end

function LimbChain:IterateUntilGoal(targetPosition,tolerance,InputtedMaxBreakCount)

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
    Optional Parameter floorNormal to make feet upright to the floor its being raycasted to
    Feet implementation is real buggy tho and is not automatic implementation
    And it only works for my mech model
]]
function LimbChain:UpdateMotors(floorNormal)

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
        
        --Changes the current motor6d through c0
        self.Motor6DTable[i].C0 = undoPreviousLimbCF*rotateLimbCF

            --Testing to keep foot appendage upright
            --Seems to work for now to make the feet look down visually
            --doesn't adhere to constraints but it works for now
            --I really need help making the feet match the surface
            if self.IncludeAppendage == true and i == iterateUntil then
                local previousLimbPart = self.Motor6DTable[#self.Motor6DTable].Part0
                local previousLimbCF = previousLimbPart.CFrame
                local empty = Vector3.new()
                --self.Motor6DTable[#self.Motor6DTable].C0 = CFrame.new()

                --Make feet point upright to floor
                --If not inputted then make feet points up towards sky
                local upright = floorNormal
                if upright == nil then
                    upright = Vector3.new(0,1,0)
                end

                --Obtain the CFrame operations needed to rotate the limb to the goal
                local undoPreviousLimbCF = previousLimbCF:Inverse()*CFrame.new(motorPosition)
                local rotateLimbCF =CFrame.fromMatrix(empty,previousLimbCF.RightVector,upright)
            
                --Changes the current motor6d through c0
                self.Motor6DTable[i].C0 = undoPreviousLimbCF*rotateLimbCF
    
            end
        end

    
    end

end

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

            --Testing to keep foot appendage upright
            --Seems to work for now to make the feet look down visually
            --doesn't adhere to constraints but it works for now
            --I really need help making the feet match the surface
            if self.IncludeAppendage == true and i == iterateUntil then
                local previousLimbPart = self.Motor6DTable[#self.Motor6DTable].Part0
                local previousLimbCF = previousLimbPart.CFrame
                local empty = Vector3.new()
                --self.Motor6DTable[#self.Motor6DTable].C0 = CFrame.new()

                --Make feet point upright to floor
                --If not inputted then make feet points up towards sky
                local upright = floorNormal
                if upright == nil then
                    upright = Vector3.new(0,1,0)
                end

                --Obtain the CFrame operations needed to rotate the limb to the goal
                local undoPreviousLimbCF = previousLimbCF:Inverse()*CFrame.new(motorPosition)
                local rotateLimbCF =CFrame.fromMatrix(empty,previousLimbCF.RightVector,upright)
                            
                --This time instead of changing the current motor C0 stores them and adds them in the table
                MotorsC0Store[#MotorsC0Store + 1] = undoPreviousLimbCF*rotateLimbCF
            end
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

    --Creates a part for each limb vector
    for i=1,#limbVectors,1 do
    local part = Instance.new("Part")
    part.Anchored = true
    part.Name = "LimbVector:"..i
    part.BrickColor = BrickColor.random()
    part.Parent = workspace
    end
    self.DebugMode = true
end

return LimbChain
