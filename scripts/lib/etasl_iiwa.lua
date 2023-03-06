-- ==============================================================================
-- Author: Yudha Pane
-- email: <yudha.pane@kuleuven.be>
-- File to deploy the driver of the KUKA LBR iiwa
-- KU Leuven 2020
-- ==============================================================================
local M = {}

function deploy_driver(timefreq)
      -- deploy iiwa robot
      iiwa_driver_dir = rtt.provides("ros"):find("iiwa_fri_rtt")
      ros:import("iiwa_fri_rtt")
      depl:loadComponent("iiwa", "FRI::FRIDriver")
      depl:loadService("iiwa", "marshalling")
      iiwa = depl:getPeer("iiwa")
      iiwa:provides("marshalling"):updateProperties(iiwa_driver_dir.."/scripts/configuration/iiwa.cpf")
      iiwa:configure()
      iiwa:start()

end


function connect_ports_driver(etasl_component,timefreq)
      compname = etasl_component:getName()

      cp = rtt.Variable("ConnPolicy")
      jn = joint_names()
      depl:connect(compname..".JointVelocityCommand", "iiwa.JointVelocityCommand", cp)
      depl:connect("iiwa.joint_states", compname..".JointStateiiwa", cp)
end

function joint_names()
      local v = rtt.Variable("strings")
      v:fromtab({"lbr_iiwa_joint_1", "lbr_iiwa_joint_2", "lbr_iiwa_joint_3",
             "lbr_iiwa_joint_4", "lbr_iiwa_joint_5", "lbr_iiwa_joint_6",
             "lbr_iiwa_joint_7"})
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
      jointstates_component:add_controller_jointstate_inputport(compname, "JointStateiiwa", "Joint state value for the controller", jn)
      etasl_component:add_controller_motion_control_msgs_joint_velocities_outputport("JointVelocityCommand", "Joint velocity values for the iiwa", jn)
end


-- export functions
M.deploy_driver = deploy_driver
M.connect_ports_driver = connect_ports_driver
M.joint_names = joint_names
M.initial_joints_states = initial_joints_states
M.create_etasl_ports = create_etasl_ports
M.home_joint_positions = home_joint_positions
M.robot_name = "kuka_iiwa"

return M
