x=PHCCScriptLua();
doasend='\007'
devaddr='\080'   -- 80dec == 0x50
s="Firmware Version V0.0.1"
s="_______________________"

function dly()
	for j=1,50000 do
		nix=j
	end
end	     

-- x:sendPHCC('\000')

for i=1,string.len(s) do
	c = string.sub(s,i,i)
	x:sendPHCC(doasend)
	dly()
	x:sendPHCC(devaddr)
	dly()
	x:sendPHCC(string.char(i-1))
	dly()
	x:sendPHCC(c)
	print(c)
	dly()
end
x:sendPHCC(doasend .. devaddr .. '\063' .. '\000')   -- 63dec == 0x3f

