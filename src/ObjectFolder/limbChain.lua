local Package = script:FindFirstAncestorOfClass("Folder")
local Object = require(Package.BaseRedirect)

local limbChain = Object.new("limbChain")
--local limbChain = Object.newExtends("limbChain",?)

function limbChain.new(limbPart)
	local obj = limbChain:make()
	--local obj = limbChain:super()
	obj.Part = limbPart
	return obj
end

return limbChain