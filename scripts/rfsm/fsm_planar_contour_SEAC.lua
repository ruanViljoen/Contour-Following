-- ==============================================================================
-- Author: Ruan Viljoen
-- email: <ruan.viljoen@kuleuven.be>
-- Author: Johan Ubbink
-- email: <johan.ubbink@kuleuven.be>
-- Definition of the state machine
-- KU Leuven 2023
-- ==============================================================================
require("rtt")
require("rttlib")

tc = rtt.getTC()
depl = tc:getPeer("Deployer")
etaslcore = depl:getPeer("etaslcore")
solver = depl:getPeer("solver")
simulation = tc:getProperty("simulation"):get()
robot_etasl_dir = tc:getProperty("robot_etasl_dir"):get()
depl_robot_file = tc:getProperty("depl_robot_file"):get()
robot = require(depl_robot_file)
joint_pos = robot.home_joint_positions()
home_pos = {3.1421215534210205, -1.9488166014300745, 2.467452049255371, -2.0881460348712366, -1.570813004170553, -0.7878339926349085}

function driver_particularities()
    if robot.robot_name == "franka_panda" and not simulation then
        local panda = depl:getPeer("panda")
        panda:low_level_velocity()
    end

end

etasl_application_dir = rtt.provides("ros"):find("planar_contour_following")

return rfsm.state {

-- ==============================================================================================================
-- Define rFSM states  ==========================================================================================
-- ==============================================================================================================

    configured = rfsm.state {
        entry = function()
            rtt.sleep(2, 0) -- Sleep for 2 seconds
        end
    },

    go_to_home_position = rfsm.state {
        entry = function()
            -- Create etasl controller component using specification file
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/contour_default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/move_jointspace_trap.lua")
            etaslcore:set_etaslvar("maxvel", 0.2)
            etaslcore:set_etaslvar("maxacc", 0.1)
            if robot.is_continuous_joints then -- If the function exists it modifies it. Otherwise the defaults are used (non continuous)
                for i = 1, #joint_pos do
                    etaslcore:set_etaslvar("continuous_j" .. i, robot.is_continuous_joints()[i])
                end
            end
            for i = 1, #joint_pos do
                etaslcore:set_etaslvar("end_j" .. i, home_pos[i])
            end

            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()

        end,
        exit = function()
            etaslcore:stop()
            etaslcore:cleanup()
        end
    },

    move_up = rfsm.state {
        entry = function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/contour_default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/moving_in_task_frame_coordinates.lua")
            etaslcore:set_etaslvar("global.maxvel", 0.1)
            etaslcore:set_etaslvar("global.maxacc", 0.02)
            etaslcore:set_etaslvar("global.eq_r", 0.08)
            etaslcore:set_etaslvar("global.delta_x", 0) -- in TCP frame direction
            etaslcore:set_etaslvar("global.delta_y", 0) -- in TCP frame direction
            etaslcore:set_etaslvar("global.delta_z", 0.1) -- in TCP frame direction
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()
        end,
        exit = function()
            etaslcore:stop()
            etaslcore:cleanup()
        end
    },

    move_down = rfsm.state {
        entry = function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/contour_default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/moving_in_task_frame_coordinates.lua")
            etaslcore:set_etaslvar("global.maxvel", 0.1)
            etaslcore:set_etaslvar("global.maxacc", 0.01)
            etaslcore:set_etaslvar("global.eq_r", 0.08)
            etaslcore:set_etaslvar("global.delta_x", 0) -- in TCP frame direction
            etaslcore:set_etaslvar("global.delta_y", 0) -- in TCP frame direction
            etaslcore:set_etaslvar("global.delta_z", -0.1) -- in TCP frame direction (0.085 for circle)
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()
        end,
        exit = function()
            etaslcore:stop()
            etaslcore:cleanup()
        end
    },

    contour_following_task = rfsm.state {
        entry = function()

            -- Create etasl controller component using specification file
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/contour_default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/contour_following_SEAC.lua")
            etaslcore:set_etaslvar("F_des", 10.0)
            etaslcore:set_etaslvar("v_des", 0.035)
            etaslcore:set_etaslvar("estimated_stiffness", 2500.0)
            etaslcore:set_etaslvar("tgt_z", 0.035) -- 0.045 for circle
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()

        end,
        exit = function()
            etaslcore:stop()
            etaslcore:cleanup()
        end
    },

    move_away_from_contour = rfsm.state {
        entry = function()

            -- Create etasl controller component using specification file
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/contour_default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir .. "/scripts/etasl/controller_specification_files/move_away_from_contour.lua")
            etaslcore:set_etaslvar("F_des", 10.0)
            etaslcore:set_etaslvar("v_des", 0)
            etaslcore:set_etaslvar("estimated_stiffness", 2500.0)
            etaslcore:set_etaslvar("tgt_z", 0.035)
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()

        end,
        exit = function()
            etaslcore:stop()
            etaslcore:cleanup()
        end
    },
    
-- ==============================================================================================================
-- Define rFSM state sequence  ==================================================================================
-- ==============================================================================================================

    rfsm.trans {
        src = "initial",
        tgt = "configured"
    },
    rfsm.trans {
        src = "configured",
        tgt = "go_to_home_position",
        events = {}
    },
    rfsm.trans {
        src = "go_to_home_position",
        tgt = "move_down",
        events = {"e_finished@etaslcore"}
    },
    rfsm.trans {
        src = "move_down",
        tgt = "contour_following_task",
        events = {"e_finished@etaslcore"}
    },
    rfsm.trans {
        src = "contour_following_task",
        tgt = "move_away_from_contour",
        events = {"e_finished@etaslcore"}
    },
    rfsm.trans {
        src = "move_away_from_contour",
        tgt = "move_up",
        events = {"e_finished@etaslcore"}
    },

}
