local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

local MathPlane = Object.new("MathPlane")

function MathPlane.new(NormalVector,Point)
	local obj = MathPlane:make()

    --Stores the inputted NormalVector
    obj.NormalVector = NormalVector

    --Stores the point on the plane
    obj.Point = Point

    --Obtains the scalar dot product of the plane
    obj.Scalar = Point:Dot(NormalVector)

	return obj
end

return MathPlane