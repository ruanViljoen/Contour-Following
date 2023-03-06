-- ==============================================================================
-- Author: Santiago Iregui
-- email: <santiago.iregui@kuleuven.be>
-- File related to the deployment of the Kinova Gen3 robot
-- KU Leuven 2020
-- ==============================================================================
local M = {}

function deploy_driver(timefreq)
  depl:import("kinova_driver")
  depl:loadComponent("kin_driver", "kinova_gen3")
  kin = depl:getPeer("kin_driver")
  kin:getProperty("ip_address"):set("192.168.1.10")
  kin:getProperty("debug_mode"):set(false) --needs to be changed before kin:configure()
  kin:getProperty("setpoints_frequency"):set(1/timefreq)

  depl:setActivity("kin_driver", 0, 99, rtt.globals.ORO_SCHED_RT)

  kin:configure()
  kin:set_servoing_mode(1)
end


function connect_ports_driver(comp,timefreq)
      compname = comp:getName()
      depl:connect(compname..".jointvel","kin_driver.control_joint_velocities",cp )

      depl:connect("kin_driver.sensor_joint_angles",compname..".jointpos",cp )
      depl:connect("kin_driver.event_port","Supervisor.events",cp )

      kin:start()
      kin:start_sending_setpoints()
end

function joint_names()
      local v = rtt.Variable("strings")
      local joint_names
      if simulation then
        joint_names={"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7","finger_joint"};
      else
        joint_names={"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"};
      end
      v:fromtab(joint_names)
      return v
end

function initial_joints_states()
  -- Only used for simulation. With the real robot eTaSL uses the sensed initial joints
      local v = rtt.Variable("array")
      v:fromtab({0,-0.34911857503313,3.1415510546877,-2.5482863919379,-2.8762139282712e-05,-0.87269071008935,1.5707952347356,0})
      return v
end

function home_joint_positions()
  -- If it is in simulation, the robot has 8 joints (including the gripper). If not, due to the robot definition, eTaSL ignores the gripper joint and it is operated through discrete signals
      local pos = {0.11770952938807,0.18931905865823,3.24113527544,-2.0823964608961,-0.021246698814222,-0.80967233028978,1.5798659612537}
      if simulation then
        pos[8] = 0
      end
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


function is_continuous_joints()
      local continuous = {1,0,1,0,1,0,1}
      if simulation then
        continuous[8] = 0 -- The last joint in simulation is the gripper
      end
      return continuous
end



-- export functions
M.deploy_driver = deploy_driver
M.connect_ports_driver = connect_ports_driver
M.joint_names = joint_names
M.initial_joints_states = initial_joints_states
M.create_etasl_ports = create_etasl_ports
M.home_joint_positions = home_joint_positions
M.robot_name = "kinova_gen3"
M.is_continuous_joints = is_continuous_joints

return M
