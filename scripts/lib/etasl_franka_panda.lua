-- ==============================================================================
-- Author: Santiago Iregui
-- email: <santiago.iregui@kuleuven.be>
-- File related to the deployment of the Kinova Gen3 robot
-- KU Leuven 2020
-- ==============================================================================
local M = {}

function deploy_driver(timefreq)

print("Make sure you are using a real time kernel when executing the Franka Panda Driver")
depl:import("orocos_franka_panda")
-- ros:import("orocos_franka_panda")
depl:loadComponent("panda", "FrankaComponent")
panda=depl:getPeer("panda")
panda:getProperty('ip_address'):set("172.16.0.2")


-- =======================Uncomment for Joint Impedance====================================
joint_impedance = rtt.Variable("array")
joint_impedance:fromtab{5000, 5000, 5000, 5000, 5000, 5000, 5000}
panda:getProperty('joint_impedance'):set(joint_impedance)
panda:getProperty('impedance_mode'):set("joint")


--=======================Uncomment for Cartesian Impedance====================================
-- cartesian_impedance = rtt.Variable("array")
-- cartesian_impedance:fromtab{3000, 3000, 2000, 100, 100, 100}
-- panda:getProperty('cartesian_impedance'):set(cartesian_impedance)
-- panda:getProperty('impedance_mode'):set("cartesian")



panda:getProperty('event_stop_loop'):set("e_finished@etaslcore")



end


function connect_ports_driver(comp,timefreq)

      compname = comp:getName()
      depl:setActivity("panda", 0, 99, rtt.globals.ORO_SCHED_RT)

      cp=rtt.Variable("ConnPolicy")

      -- Joint and velocity values
      depl:connect(compname..".jointvel","panda.control_joint_velocities",cp)
      depl:connect("panda.sensor_joint_angles",compname..".jointpos",cp)
      -- depl:connect("panda.events_port",compname..".events",cp)
      depl:connect("panda.events_port",compname..".eventPort",cp)

      panda:configure()
      panda:error_recovery()
      -- panda:low_level_velocity()
end

function joint_names()
      local v = rtt.Variable("strings")
      local joint_names
      joint_names={"panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"}
      v:fromtab(joint_names)
      return v
end

function initial_joints_states()
  -- Only used for simulation. With the real robot eTaSL uses the sensed initial joints
      local v = rtt.Variable("array")
      v:fromtab({-0.13499072678884,-0.83698763193164,0.062471039650238,-2.8514912586881,0.033920903435334,2.0334098398421,0.5})
      return v
end

function home_joint_positions()
        pos = {-0.13499072678884,-0.83698763193164,0.062471039650238,-2.8514912586881,0.033920903435334,2.0334098398421,0.73794362216389}
      return pos
end

function create_etasl_ports(etasl_component,jointstates_component)
      compname = etasl_component:getName()
      jn = joint_names()
      etasl_component:add_controller_inputport("jointpos","Joint position values",jn)
      etasl_component:add_controller_outputport("jointvel","Joint velocity values",jn)
      jointstates_component:add_controller_jointstate_outputport(compname,"joint_state","Joint state value for the controller",jn)
      etasl_component:add_controller_pos_outputport("jointpos_out","Joint Position values",jn)
end



-- export functions
M.deploy_driver = deploy_driver
M.connect_ports_driver = connect_ports_driver
M.joint_names = joint_names
M.initial_joints_states = initial_joints_states
M.create_etasl_ports = create_etasl_ports
M.home_joint_positions = home_joint_positions
M.robot_name = "franka_panda"

return M
