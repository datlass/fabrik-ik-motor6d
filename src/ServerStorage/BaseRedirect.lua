local Package = script:FindFirstAncestor("LimbChain")

local baseModule = Package.Base
--^^ Simply adjust the variable so it matches the Base's location. ^^

return require(baseModule)
