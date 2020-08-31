local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

local MathPlane = Object.new("MathPlane")
--local MathPlane = Object.newExtends("MathPlane",?)

function MathPlane.new()
	local obj = MathPlane:make()
	--local obj = MathPlane:super()
	
	return obj
end

return MathPlane