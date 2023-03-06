-- ==============================================================================
-- Author: Ruan Viljoen
-- email: <ruan.viljoen@kuleuven.be>
-- Simulates second order system
-- KU Leuven 2022
-- ==============================================================================

-- The purpose of this component is to model a discrete-time lead compensator.

require("rttlib")
require("math")

-- depl:import("rtt_sensor_msgs")
-- depl:import("rtt_geometry_msgs")
-- depl:import("rtt_sensor_msgs")
-- depl:import('kdl_typekit')

tc=rtt.getTC()

rot_mat = rtt.provides("KDL"):provides("Rotation")

iface_spec = {
   ports={
    { name='u', datatype='array', type='in', desc="input to the second order system" },
    { name='y', datatype='array', type='out', desc="output of the second order system" },
    { name='joint_velocity_commands', datatype='/motion_control_msgs/JointVelocities', type='out', desc="output of the second order system in JointVelocities msgs" }
   },
   properties={
      { name='properties', datatype='array', desc="wn,alpha,Ts" },
   }
}


iface=rttlib.create_if(iface_spec) 

output_vals=rtt.Variable("array")
properties_vals=rtt.Variable("array")
wn = rtt.Variable("double")
alpha = rtt.Variable("double")
Ts = rtt.Variable("double")
a1 = rtt.Variable("double")
a2 = rtt.Variable("double")
b1 = rtt.Variable("double")
b2 = rtt.Variable("double")

y = rtt.Variable("array")
y_old = rtt.Variable("array")

u = rtt.Variable("array")
u_old = rtt.Variable("array")

joint_velocity_commands = rtt.Variable("/motion_control_msgs/JointVelocities")

-- The Lua component starts its life in PreOperational, so
-- configureHook can be used to set stuff up.
function configureHook() 
    return true
end

function startHook()
    y_old:fromtab({0,0,0,0,0,0}) -- for the 7 joints
    u:fromtab({0,0,0,0,0,0}) -- for the 7 joints
    u_old:fromtab({0,0,0,0,0,0}) -- for the 7 joints
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
        alpha = properties_vals[1]
        beta = 0.3/(math.sqrt(alpha)*wn)
        Ts = properties_vals[2]

        a1 = beta - Ts
        a2 = - beta - Ts

        b1 = alpha*beta - Ts
        b2 = - alpha*beta - Ts

        y:fromtab({0,0,0,0,0,0})

        for i=0,5 do
            y[i] =  ( a1*u_old[i] + a2*u[i+1] - b1*y_old[i] ) / b2

            y_old[i] = y[i]
    
            u_old[i] = u[i+1]
        end

        joint_velocity_commands:fromtab({velocities={y[0],y[1],y[2],y[3],y[4],y[5],y[6]}, names={"iiwa1_joint_1", "iiwa1_joint_2", "iiwa1_joint_3", "iiwa1_joint_4", "iiwa1_joint_5", "iiwa1_joint_6", "iiwa1_joint_7"}})

        iface.ports.joint_velocity_commands:write(joint_velocity_commands)

    end


end


function cleanupHook()
    rttlib.tc_cleanup()
end