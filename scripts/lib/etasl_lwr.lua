-- ==============================================================================
-- Author: Yudha Pane
-- email: <yudha.pane@kuleuven.be>
-- File to deploy the driver of the KUKA LWR4
-- KU Leuven 2020
-- ==============================================================================
local M = {}

function deploy_driver(timefreq)
      this_package_dir = rtt.provides("ros"):find("planar_contour_following")
      ros:import("rtt_lwr_fri_msgs")
      lwr_driver_dir = rtt.provides("ros"):find("lwr_fri")
      ros:import("lwr_fri")
      depl:loadComponent("lwr", "lwr_fri::FRIComponent")
      depl:loadService("lwr", "marshalling")
      lwr = depl:getPeer("lwr")
      lwr:provides("marshalling"):updateProperties(this_package_dir.."/scripts/config/lwr.cpf")
      lwr:configure()
      lwr:start()

end


function connect_ports_driver(etasl_component,timefreq)
      compname = etasl_component:getName()

      cp = rtt.Variable("ConnPolicy")
      jn = joint_names()
      depl:connect(compname..".JointVelocityCommand", "lwr.JointVelocityCommand", cp)
      depl:connect("lwr.JointState", compname..".JointState", cp)
end

function joint_names()
      local v = rtt.Variable("strings")
      v:fromtab({"lwr_arm_joint_0", "lwr_arm_joint_1", "lwr_arm_joint_2", 
             "lwr_arm_joint_3", "lwr_arm_joint_4", "lwr_arm_joint_5", 
             "lwr_arm_joint_6"})
      return v
end

function initial_joints_states()
  -- Only used for simulation. With the real robot eTaSL uses the sensed initial joints
      local v = rtt.Variable("array")
      v:fromtab({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
      return v
end

function home_joint_positions()
      pos = { 0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -90/180*math.pi, 0/180*math.pi, 90/180*math.pi, 0/180*math.pi }
      return pos
end

function create_etasl_ports(etasl_component,jointstates_component)
      compname = etasl_component:getName()
      jn = joint_names()
      etasl_component:add_controller_inputport("jointpos","Joint position values",jn)
      etasl_component:add_controller_outputport("jointvel","Joint velocity values",jn)
      jointstates_component:add_controller_jointstate_outputport(compname,"joint_state","Joint state value for the controller",jn)
      etasl_component:add_controller_pos_outputport("jointpos_out","Joint Position values",jn)
      jointstates_component:add_controller_jointstate_inputport(compname, "JointState", "Joint state value for the controller", jn)
      etasl_component:add_controller_motion_control_msgs_joint_velocities_outputport("JointVelocityCommand", "Joint velocity values for the lwr", jn)
end


-- export functions
M.deploy_driver = deploy_driver
M.connect_ports_driver = connect_ports_driver
M.joint_names = joint_names
M.initial_joints_states = initial_joints_states
M.create_etasl_ports = create_etasl_ports
M.home_joint_positions = home_joint_positions
M.robot_name = "kuka_lwr"

return M
