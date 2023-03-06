require("xmlrpc.http")
require("rttlib")
tc=rtt.getTC();
xmlrcp_server_address = "http://127.0.0.1:40408" -- Location of the xmlrcp address, can be changed in the config.ini file

-- The Lua component starts its life in PreOperational, so
-- configureHook can be used to set stuff up.
function configureHook()
   -- Connect to the gripper (Modbus comunication)
   local ok, res = xmlrpc.http.call(xmlrcp_server_address, "connect",true)
   if not ok then
      return false
   end
   ok, res = xmlrpc.http.call(xmlrcp_server_address, "cmdAcknowledge")
   print(ok)
   if not ok then
      return false
   end
   os.execute("sleep 1")
   xmlrpc.http.call(xmlrcp_server_address, "cmdReferencing")

   grip_action = rtt.InputPort("array", "grip_action")    -- global variable!
   tc:addEventPort(grip_action)

   cnt = 0
   return true
end

-- all hooks are optional!
--function startHook() return true end

function updateHook()
   local fs, array_in = grip_action:read()
   command = array_in[0]
   if fs=="NewData" and command==0 then
      print("Gripping")
      local ok, is_open = xmlrpc.http.call(xmlrcp_server_address, "isOpen")
      if is_open then
         -- print("Gripping with force"..array_in[1].." (0 --> 100%, 1 --> 75%, 2 --> 50%, 3 --> 25%)")
         local ok, res = xmlrpc.http.call(xmlrcp_server_address, "cmdGrip", array_in[1])
      end
   elseif fs=="NewData" and command==1 then
      local ok, is_open = xmlrpc.http.call(xmlrcp_server_address, "isOpen")
      if not is_open then
         print("Releasing part")
         local ok, res = xmlrpc.http.call(xmlrcp_server_address, "cmdRelease")
      end
   elseif fs=="NewData" and command==2 then
      local ok, res = xmlrpc.http.call(xmlrcp_server_address, "getMaximalStroke")
      if array_in[1] < res-0.5 and array_in[1] > 0.5 then
         print("Moving fingers to position "..array_in[1])
         local ok, res = xmlrpc.http.call(xmlrcp_server_address, "cmdPositioning", array_in[1])
      end
   elseif fs=="NewData" and command==3 then
      print("Setting up paramters for WP")
      local ok, res = xmlrpc.http.call(xmlrcp_server_address, "setWorkpieceParam", array_in[1], 1, array_in[2])
      local ok, res = xmlrpc.http.call(xmlrcp_server_address, "setWorkpieceParam", array_in[1], 2, array_in[3])
      local ok, res = xmlrpc.http.call(xmlrcp_server_address, "setWorkpieceParam", array_in[1], 3, array_in[4]) --grip force can be set to 0 --> 100%, 1 --> 75%, 2 --> 50%, 3 --> 25% 100% is arround 40N
      local ok, res = xmlrpc.http.call(xmlrcp_server_address, "setWorkpieceParam", array_in[1], 4, array_in[5])
   end
   -- rtt.log("data received: " .. tostring(data) .. ", flowstatus: " .. fs)
end

-- Ports and properties are the only elements which are not
-- automatically cleaned up. This means this must be done manually for
-- long living components:
function cleanupHook()
   tc:removePort("grip_action")
   grip_action:delete()
end
