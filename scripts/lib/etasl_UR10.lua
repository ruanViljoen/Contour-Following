-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- File to deploy the driver of the UR10
-- KU Leuven 2020
-- ==============================================================================
local M = {}

function deploy_driver(timefreq)
      URDriver_dir=rtt.provides("ros"):find("URDriver")

      ros:import("URDriver")
      depl:loadComponent("URDriverRT_receiver", "URDriverRT_receiver")
      URDriverRT_receiver=depl:getPeer("URDriverRT_receiver")

      depl:loadComponent("URDriver_program", "URDriver_program")
      URDriver_program=depl:getPeer("URDriver_program")


      URDriver_program:setPeriod(timefreq)

      URDriver_program:getProperty("robot_address"):set("192.168.1.102")
      URDriver_program:getProperty("my_address"):set("192.168.1.101")
      URDriverRT_receiver:getProperty("robot_address"):set("192.168.1.102")
      URDriverRT_receiver:getProperty("version_interface"):set("3.0-3.1")
      URDriver_program:getProperty("timeOut"):set(timefreq)

      URDriver_program:getProperty("program_file"):set(URDriver_dir .. "/prog.ur")

      if not URDriverRT_receiver:configure() then
      print("failed to configure URDriverRT_receiver")
      return
      end

      if not URDriverRT_receiver:start()then print("failed to start") end
      if not URDriver_program:configure() then
      print("failed to conf URDriver_program")
      return
      end

      print(">> send-program")
      if not URDriver_program:send_program()then   print("failed to send-program") end

      print(">> open-server")
      if not URDriver_program:open_server()then   print("failed to open-server")end
      if not URDriver_program:start()then   print("failed to start URDriver_program") end
      print(">> URDriver started")
end


function connect_ports_driver(comp,timefreq)
      compname = comp:getName()
      depl:connect(compname..".jointvel","URDriver_program.qdes_inport",cp)
      depl:connect("URDriverRT_receiver.q_actual_outport",compname..".jointpos",cp)

      URDriver_program:start_send_velocity()
end

function joint_names()
      local v = rtt.Variable("strings")
      v:fromtab({"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"})
      return v
end

function initial_joints_states()
  -- Only used for simulation. With the real robot eTaSL uses the sensed initial joints
      local v = rtt.Variable("array")
      v:fromtab({180/180*math.pi, -90/180*math.pi, 90/180*math.pi, -90/180*math.pi, -90/180*math.pi, 0/180*math.pi})
      return v
end

function home_joint_positions()
      pos = { 180/180*math.pi, -110/180*math.pi, 110/180*math.pi, -90/180*math.pi, -90/180*math.pi, 45/180*math.pi }
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
M.robot_name = "ur_10"

return M
