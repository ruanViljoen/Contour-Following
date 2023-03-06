-- ==============================================================================
-- Author: Ruan Viljoen
-- email: <ruan.viljoen@kuleuven.be>
-- Simulates second order system
-- KU Leuven 2022
-- ==============================================================================

-- The purpose of this component is to model a second order discrete-time system. If multiple inputs are provided then each of them is passed through a decoupled second order system

require("rttlib")
require("math")

tc=rtt.getTC()

rot_mat = rtt.provides("KDL"):provides("Rotation")

iface_spec = {
   ports={
    { name='u', datatype='array', type='in', desc="input to the second order system" },
    { name='y', datatype='array', type='out', desc="output of the second order system" }
   },
 
   properties={
      { name='properties', datatype='array', desc="wn,zeta,ts" },
   }
}


iface=rttlib.create_if(iface_spec) 

output_vals=rtt.Variable("array")
properties_vals=rtt.Variable("array")
wn = rtt.Variable("double")
zeta = rtt.Variable("double")
Ts = rtt.Variable("double")
a1 = rtt.Variable("double")
b1 = rtt.Variable("double")
b2 = rtt.Variable("double")
b3 = rtt.Variable("double")

y = rtt.Variable("array")
y_old = rtt.Variable("array")
y_old_old = rtt.Variable("array")

u = rtt.Variable("array")
u_old = rtt.Variable("array")
u_old_old = rtt.Variable("array")

-- The Lua component starts its life in PreOperational, so
-- configureHook can be used to set stuff up.
function configureHook() 
    y_old:fromtab({0,0,0,0,0,0}) -- for the 7 joints
    y_old_old:fromtab({0,0,0,0,0,0}) -- for the 7 joints
    u:fromtab({0,0,0,0,0,0}) -- for the 7 joints
    u_old:fromtab({0,0,0,0,0,0}) -- for the 7 joints
    u_old_old:fromtab({0,0,0,0,0,0}) -- for the 7 joints
    return true
end

function startHook()
    return true
end


function updateHook()

    local properties = iface.props.properties:get():totab()

    properties_vals:fromtab( properties )
    fs,u=iface.ports.u:read()
    -- fs1,force_sensor=iface.ports.force_sensor_frame:read()

    if fs~='NoData' then 
        local u = u:totab()
        -- local dt = tc:getPeriod() 

        wn = properties_vals[0]
        zeta = properties_vals[1]
        Ts = properties_vals[2]

        a1 = Ts*Ts*wn*wn

        b1 = Ts*Ts*wn*wn + 1 - 2*zeta*wn*Ts
        b2 = 2*Ts*Ts*wn*wn - 2
        b3 = 1 + Ts*Ts*wn*wn + 2*zeta*Ts*wn       

        y:fromtab({0,0,0,0,0,0})

        for i=0,5 do
            y[i] =  ( a1*u_old_old[i] + 2*a1*u_old[i] + a1*u[i+1] - b1*y_old_old[i] - b2*y_old[i] ) / b3

            y_old_old[i] = y_old[i]
            y_old[i] = y[i]
    
            u_old_old[i] = u_old[i]
            u_old[i] = u[i+1]
        end
        iface.ports.y:write(y)
    end


end


function cleanupHook()
    rttlib.tc_cleanup()
end