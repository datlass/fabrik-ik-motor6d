local function readCFrame(data) 
	if typeof(data)=="CFrame" then
		print("Position: ",data.p)
		print("UpVector: ",data.UpVector)
		print("RightVector: ",data.RightVector)
		print("LookVector: ",data.LookVector)
	end
end