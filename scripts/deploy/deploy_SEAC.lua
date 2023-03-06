-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- KU Leuven 2020
-- ==============================================================================
-- ==============================================================================
-- Adapted by Rob Eyskens
-- email: <rob.eyskens@student.kuleuven.be>
-- KU Leuven 2022
-- ==============================================================================
-- Adapted also by Ruan Viljoen
-- email: <ruan.viljoen@kuleuven.be>
-- KU Leuven 2022
-- ==============================================================================

require "rttlib"
require "rttros"
require "deployer_utils"

-- ====================================== User Parameters =========================================

-- robot_name = "kinova_gen3"
-- robot_name = "franka_panda"
-- robot_name = "kuka_iiwa"
-- robot_name = "dual_kuka_iiwa"
robot_name = "ur_10"

-- use_jr3: Use it if you want to integrate a jr3 wrench sensor
use_jr3 = false
use_gripper = false

-- freq: Frequency [Hz] at which the eTaSL and the corresponding components run.
freq = 200

-- ====================================== Standard deployment stuff =========================================
rtt.setLogLevel("Warning")

gs = rtt.provides()
tc = rtt.getTC()

if tc:getName() == "lua" then
    depl = tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
    depl = tc
end

depl:import("rtt_ros")
ros = gs:provides("ros")
ros:import("etasl_rtt")
ros:import("rtt_rospack")
rttlib.color = true

depl:import("rtt_sensor_msgs")
depl:import("rtt_geometry_msgs")
depl:import("rtt_sensor_msgs")
depl:import('kdl_typekit')

etasl_application_dir = rtt.provides("ros"):find("planar_contour_following")
dofile(etasl_application_dir .. "/scripts/lib/deployer_utils.lua")
robot_def_dir = etasl_application_dir .. "/scripts/etasl/robot_def"

-- The following will make run always in simulation, unless you provide "deploy_robot as the first argument"
-- Example to run in real robot: rttlua -i deploy_general.lua "real_robot"
if arg[1] == "real_robot" then
    simulation = false
    use_jr3 = true
else
    simulation = true
    use_gripper = false
end

cp = rtt.Variable("ConnPolicy")

-- ====================================== Robot Hardware definition =========================================
depl_robot_file, robot_etasl_dir = determine_robot(robot_name)
robot = dofile(etasl_application_dir .. "/scripts/lib/" .. depl_robot_file .. ".lua")





-- ==============================================================================================================
-- eTaSL CONTROLLER =============================================================================================
-- ==============================================================================================================

ros:import("etasl_solver_qpoases")
depl:loadComponent("solver", "etasl_solver_qpoases")
solver = depl:getPeer("solver")

ros:import("etasl_iohandler_jointstate")
depl:loadComponent("jointstate", "Etasl_IOHandler_Jointstate")
jointstate = depl:getPeer("jointstate")

ros:import("etasl_rtt")
depl:loadComponent("etaslcore", "etasl_rtt")
etaslcore = depl:getPeer("etaslcore")

depl:connectPeers("etaslcore", "solver")
depl:connectPeers("etaslcore", "jointstate")

robot.create_etasl_ports(etaslcore, jointstate)

-- ====================================== eTaSL controller I/O
etaslcore:add_etaslvar_inputport("desired_velocities", "", s {"vx_d", "vy_d"}, d {})
etaslcore:add_etaslvar_inputport("wrench_array", "Vector with wrench values input port", s {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"}, d {})
etaslcore:add_etaslvar_inputport("disturbances","Estimated disturbances", s{"dot_xu_force", "dot_xu_force"}, d{})
etaslcore:add_etaslvar_inputport("xu","Estimated disturbances", s{"xu_force", "xu_x", "xu_y"}, d{})
etaslcore:add_etaslvar_deriv_inputport("dot_xu","Estimated disturbances", s{"xu_force", "xu_x", "xu_y"})

etaslcore:add_etaslvar_outputport("tf_pose", "Executed pose of the task frame", s {"x_tf", "y_tf", "z_tf", "roll_tf", "pitch_tf", "yaw_tf"})
etaslcore:add_etaslvar_outputport("TCP_pose", "Executed pose of the TCP frame", s {"x_TCP", "y_TCP", "z_TCP", "roll_TCP", "pitch_TCP", "yaw_TCP"})
etaslcore:add_etaslvar_outputport("frame_tcp", "Executed pose of the TCP frame", s {"x_TCP", "y_TCP", "z_TCP", "roll_TCP", "pitch_TCP", "yaw_TCP"})
etaslcore:add_etaslvar_outputport("ft_sensor_pose", "Executed pose of force sensor frame", s {"x_fs", "y_fs", "z_fs", "roll_fs", "pitch_fs", "yaw_fs"})
etaslcore:add_etaslvar_outputport("force_tf", "Force in task frame", s{"Fx_out", "Fy_out", "Fz_out"})
etaslcore:add_etaslvar_outputport("torque_tf", "Torque in task frame", s{"Tx_out", "Ty_out", "Tz_out"})
etaslcore:add_etaslvar_outputport("wrench_array_out", "Vector with wrench values output port", s{"Fx_sensor", "Fy_sensor", "Fz_sensor", "Tx_sensor", "Ty_sensor", "Tz_sensor"})
etaslcore:add_etaslvar_outputport("controller_debug", "", s{"delta", "xf", "yaw_TCP", "yaw_tf", "xu_theta", "xu_normal"})
etaslcore:add_etaslvar_deriv_outputport("dot_controller_debug", "", s{"delta", "xf", "yaw_TCP", "yaw_tf", "xu_theta", "xu_normal"})
etaslcore:add_etaslvar_outputport("y_ICTF", "", s{"y_ICTF", "y_ICTF"})
-- etaslcore:add_etaslvar_outputport("ff_activation", "", s{"ff_tracking", "ff_force"})
etaslcore:add_etaslvar_deriv_outputport("dot_y_ICTF", "", s{"y_ICTF", "y_ICTF"})

-- ==============================================================================================================
-- ==============================================================================================================
-- ==============================================================================================================

-- deploy lead compensator:
depl:loadComponent("lead_compensator", "OCL::LuaComponent")
lead_compensator = depl:getPeer("lead_compensator")
lead_compensator:exec_file(etasl_application_dir.."/scripts/components/lead_compensator.lua")
wn = rtt.Variable("double")
wn = 20.0 -- relative to torso
alpha = rtt.Variable("double")
alpha = 0.1 -- relative to torso
Ts = rtt.Variable("double")
Ts = 0.005
properties = rtt.Variable("array")
properties:fromtab({wn,alpha,Ts})
lead_compensator:getProperty("properties"):set(properties)
depl:connect("etaslcore.jointvel","lead_compensator.u",cp)


if simulation then

    -- When in simulation, there are three additional OROCOS components that are required:
    -- -> A second order system to model the internal motion controllers of the robot
    -- -> A integrator, to generate the robot position. This is not included in the model of the robot motion controllers, because 
    --    sometimes it is convenient to only use the integrator, and bypass the second order system model.
    -- -> The force simulator, which works for simple circular contour
    
    -- deploy second order system:
    depl:loadComponent("second_order_system", "OCL::LuaComponent")
    second_order_system = depl:getPeer("second_order_system")
    second_order_system:exec_file(etasl_application_dir.."/scripts/components/second_order_system.lua")
    wn = rtt.Variable("double")
    wn = 400.0 -- relative to torso
    zeta = rtt.Variable("double")
    zeta = 0.75 -- relative to torso
    Ts = rtt.Variable("double")
    Ts = 0.005
    properties = rtt.Variable("array")
    properties:fromtab({wn,zeta,Ts})
    second_order_system:getProperty("properties"):set(properties)

    -- depl:connect("lead_compensator.y","second_order_system.u",cp )
    depl:connect("etaslcore.jointvel","second_order_system.u",cp )

    -- deploy simulated robot:
    depl:loadComponent("simrobot", "OCL::LuaComponent")
    simrobot = depl:getPeer("simrobot")
    simrobot:exec_file(etasl_application_dir .. "/scripts/components/simple_robot_sim.lua")
    init_jnts = robot.initial_joints_states()
    simrobot:getProperty("initial_position"):set(init_jnts)
    depl:connect("second_order_system.y", "simrobot.jointvel", cp)
    depl:connect("simrobot.jointpos", "etaslcore.jointpos", cp)

    -- deploy force simulation: 
    depl:loadComponent("forcesim", "OCL::LuaComponent")
    forcesim = depl:getPeer("forcesim")
    forcesim:exec_file(etasl_application_dir .. "/scripts/components/force_simulation.lua")
    init_for = rtt.Variable("array")
    init_for:fromtab({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
    forcesim:getProperty("initial_forces"):set(init_for)
    middlepointx = rtt.Variable("double")
    middlepointx = 0.5
    middlepointy = rtt.Variable("double")
    middlepointy = 0.1
    circle_rad = rtt.Variable("double")
    circle_rad = 0.10
    stiffness = rtt.Variable("double")
    stiffness = 5000.0
    tool_length = rtt.Variable("double")
    tool_length = 0.102
    object_char = rtt.Variable("array")
    object_char:fromtab({middlepointx, middlepointy, circle_rad, stiffness, tool_length})
    forcesim:getProperty("object_char"):set(object_char)
    depl:connect("etaslcore.TCP_pose", "forcesim.task_frame", cp)
    depl:connect("etaslcore.ft_sensor_pose", "forcesim.force_sensor_frame", cp)
    depl:connect("forcesim.force", "etaslcore.wrench_array", cp)
    depl:stream("etaslcore.wrench_array", ros:topic("/force_new"))

    no_fbsched = true -- This was an attempt at improving the performance of the system by enforcing a synchronous order of execution of the components. It didn't seems to make much of a difference...

    depl:setActivity("simrobot", 1 / freq, 50, rtt.globals.ORO_SCHED_RT)

    if no_fbsched then
        depl:setActivity("second_order_system", 1/freq, 50, rtt.globals.ORO_SCHED_RT)
        depl:setActivity("forcesim", 1 / freq, 50, rtt.globals.ORO_SCHED_RT)
    
    else

        ros:import("fbsched")
        depl:loadComponent("fbs", "FBSched")
        fbs = depl:getPeer("fbs")
        sched_order = fbs:getProperty("sched_order")
        depl:connect("simrobot.triggerPort", "fbs.trigger", cp)
        depl:setMasterSlaveActivity("fbs", "forcesim")
        depl:setMasterSlaveActivity("fbs", "etaslest")
        depl:setMasterSlaveActivity("fbs", "etaslcore")
        depl:setMasterSlaveActivity("fbs", "lead_compensator")
        depl:setMasterSlaveActivity("fbs", "second_order_system")
        depl:connectPeers("forcesim", "fbs")
        depl:connectPeers("etaslest", "fbs")
        depl:connectPeers("etaslcore", "fbs")
        depl:connectPeers("lead_compensator", "fbs")
        depl:connectPeers("second_order_system", "fbs")
        sched_order:get():resize(5)
        sched_order[0] = "forcesim"
        sched_order[1] = "etaslest"
        sched_order[2] = "etaslcore"
        sched_order[3] = "lead_compensator"
        sched_order[4] = "second_order_system"

        fbs:configure()
        fbs:start()

    end

    simrobot:configure()
    simrobot:start()
    forcesim:configure()
    forcesim:start()
    second_order_system:configure()
    second_order_system:start()

else
    -- ====================================== Real robot
    robot.deploy_driver(1 / freq)
end

depl:setActivity("etaslcore", 1/freq, 50, rtt.globals.ORO_SCHED_RT)
-- depl:setActivity("etaslest", 1/freq, 50, rtt.globals.ORO_SCHED_RT)
depl:setActivity("lead_compensator", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

-- etaslest:configure()
-- etaslest:initialize()
-- etaslest:start()
lead_compensator:configure()
lead_compensator:start()


-- ====================================== Force/Torque Sensor =========================================
if use_jr3 then
    FTsensor = dofile(etasl_application_dir .. '/scripts/lib/etasl_FT_JR3.lua')
    FTsensor.FTsensor_deployer(etaslcore, 1 / freq)
end

-- ====================================== Supervisor =========================================
depl:loadComponent("Supervisor", "OCL::LuaComponent")
sup = depl:getPeer("Supervisor")

define_property(sup, "simulation", "bool", simulation, "Boolean value to set simulation mode")
define_property(sup, "robot_etasl_dir", "string", robot_etasl_dir, "Directory of the etasl robot definition")
define_property(sup, "depl_robot_file", "string", depl_robot_file,
    "Directory of the file containing deployment of the robot")
define_property(sup, "use_jr3", "bool", use_jr3, "Boolean value to set if the force sensor is in use")


sup:exec_file(etasl_application_dir .. "/scripts/components/fsm_component.lua")
sup:getProperty("state_machine"):set(etasl_application_dir .. "/scripts/rfsm/fsm_planar_contour_SEAC.lua")
sup:addPeer(depl)
sup:configure()
sup:start()
cmd = rttlib.port_clone_conn(sup:getPort("events"))

-- connect ports:
if not simulation then
    depl:connect("etaslcore.jointvel","URDriver_program.qdes_inport",cp) -- Directly connect eTaSL output to robot motion controllers
    -- depl:connect("lead_compensator.y","URDriver_program.qdes_inport",cp) -- Connect output of lead compensator to robot motion controllers (Caution: beware of noisy signal)
    depl:connect("URDriverRT_receiver.q_actual_outport","etaslcore.jointpos",cp)

    URDriver_program:start_send_velocity()
end
depl:connect("etaslcore.eventPort", "Supervisor.events", cp)





-- ==============================================================================================================
-- Connect ports to ROS topics for online plotting and recording of data ========================================
-- ==============================================================================================================

depl:stream("etaslcore.joint_state", ros:topic("/joint_states"))
depl:stream("etaslcore.jointvel", ros:topic("/dot_q_d"))
depl:stream("etaslcore.force_tf", ros:topic("/forces"))
depl:stream("etaslcore.wrench_array_out", ros:topic("/wrench_sensor"))
depl:stream("lead_compensator.y", ros:topic("/lead_dot_q_d"))
depl:stream("etaslcore.dot_y_ICTF", ros:topic("/dot_y_ICTF"))

depl:stream("etaslcore.tf_pose", ros:topic("/frame_tf"))
depl:stream("etaslcore.frame_tcp", ros:topic("/frame_tcp"))
depl:stream("etaslcore.controller_debug", ros:topic("/controller_debug"))
depl:stream("etaslcore.dot_controller_debug", ros:topic("/dot_controller_debug"))

depl:stream("etaslcore.desired_velocities", ros:topic("/desired_velocities"))

