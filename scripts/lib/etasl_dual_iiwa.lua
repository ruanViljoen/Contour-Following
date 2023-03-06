-- ==============================================================================
-- Author: Yudha Pane
-- email: <yudha.pane@kuleuven.be>
-- File to deploy the driver of the KUKA LBR iiwa
-- KU Leuven 2020
-- ==============================================================================
local M = {}
print("-----------dual arm------------")
function deploy_driver(timefreq)
      -- deploy iiwa robot
      -- iiwa_driver_dir = rtt.provides("ros"):find("iiwa_fri_rtt")
      etasl_application_dir = rtt.provides("ros"):find("planar_contour_following")
      ros:import("iiwa_fri_rtt")
      depl:loadComponent("iiwa1", "FRI::FRIDriver")
      depl:loadService("iiwa1", "marshalling")
      iiwa1 = depl:getPeer("iiwa1")
      iiwa1:provides("marshalling"):updateProperties(etasl_application_dir.."/scripts/config/iiwa1.cpf")
      iiwa1:configure()
      iiwa1:start()

      depl:loadComponent("iiwa2", "FRI::FRIDriver")
      depl:loadService("iiwa2", "marshalling")
      iiwa2 = depl:getPeer("iiwa2")
      iiwa2:provides("marshalling"):updateProperties(etasl_application_dir.."/scripts/config/iiwa2.cpf")
      iiwa2:configure()
      iiwa2:start()

end


function connect_ports_driver(etasl_component,timefreq)
      compname = etasl_component:getName()

      cp = rtt.Variable("ConnPolicy")
      -- jn = joint_names()
      depl:connect(compname..".JointVelocityCommand_1", "iiwa1.JointVelocityCommand", cp)
      depl:connect("iiwa1.joint_states", compname..".JointStateiiwa_1", cp)

      depl:connect(compname..".JointVelocityCommand_2", "iiwa2.JointVelocityCommand", cp)
      depl:connect("iiwa2.joint_states", compname..".JointStateiiwa_2", cp)
end

function joint_names()
      local v = rtt.Variable("strings")
      v:fromtab({"iiwa1_joint_1", "iiwa1_joint_2", "iiwa1_joint_3",
             "iiwa1_joint_4", "iiwa1_joint_5", "iiwa1_joint_6",
             "iiwa1_joint_7","iiwa2_joint_1", "iiwa2_joint_2", "iiwa2_joint_3",
                    "iiwa2_joint_4", "iiwa2_joint_5", "iiwa2_joint_6",
                    "iiwa2_joint_7"})
      return v
end

function initial_joints_states()
  -- Only used for simulation. With the real robot eTaSL uses the sensed initial joints
      local v = rtt.Variable("array")
      v:fromtab({ 29.9/180*math.pi, 22.25/180*math.pi, -37/180*math.pi, -82.3/180*math.pi, -32.5/180*math.pi, 77.7/180*math.pi, 0/180*math.pi,
              0/180*math.pi, 45/180*math.pi, 0/180*math.pi, -70/180*math.pi, 0/180*math.pi, 45/180*math.pi, 0/180*math.pi })
      return v
end

function home_joint_positions()
      pos = { 0/180*math.pi, 45/180*math.pi, 0/180*math.pi, -70/180*math.pi, 0/180*math.pi, 45/180*math.pi, 0/180*math.pi,
              0/180*math.pi, 45/180*math.pi, 0/180*math.pi, -70/180*math.pi, 0/180*math.pi, 45/180*math.pi, 0/180*math.pi }
      return pos
end

function create_etasl_ports(etasl_component,jointstates_component)

      compname = etasl_component:getName()
      jn_iiwa1 = s{"iiwa1_joint_1", "iiwa1_joint_2", "iiwa1_joint_3",
               "iiwa1_joint_4", "iiwa1_joint_5", "iiwa1_joint_6",
               "iiwa1_joint_7"}

      jn_iiwa2 = s{"iiwa2_joint_1", "iiwa2_joint_2", "iiwa2_joint_3",
               "iiwa2_joint_4", "iiwa2_joint_5", "iiwa2_joint_6",
               "iiwa2_joint_7"}

      jn = joint_names()
      etasl_component:add_controller_inputport("jointpos","Joint position values",jn)
      etasl_component:add_controller_outputport("jointvel","Joint velocity values",jn)
      jointstates_component:add_controller_jointstate_outputport(compname,"joint_state","Joint state value for the controller",jn)
      etasl_component:add_controller_pos_outputport("jointpos_out","Joint Position values",jn)
      jointstates_component:add_controller_jointstate_inputport(compname, "JointStateiiwa_1", "Joint state value for the controller", jn_iiwa1)
      jointstates_component:add_controller_jointstate_inputport(compname, "JointStateiiwa_2", "Joint state value for the controller", jn_iiwa2)
      etasl_component:add_controller_motion_control_msgs_joint_velocities_outputport("JointVelocityCommand_1", "Joint velocity values for the iiwa", jn_iiwa1)
      etasl_component:add_controller_motion_control_msgs_joint_velocities_outputport("JointVelocityCommand_2", "Joint velocity values for the iiwa", jn_iiwa2)
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
