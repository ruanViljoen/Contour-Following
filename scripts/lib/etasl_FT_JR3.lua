-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Main deploy the driver of the force sensor
-- KU Leuven 2020
-- ==============================================================================

local M = {}

function FTsensor_deployer(comp,timefreq)
      compname = comp:getName()
      ros:import("jr3")
      depl:loadComponent("jr3","jr3::jr3")
      jr3 = depl:getPeer("jr3")
      depl:setActivity("jr3", timefreq, 0, rtt.globals.ORO_SCHED_RT)
      jr3:configure()
      jr3:start()

      depl:import("force_sensor")
      depl:loadComponent("force", "force_sensor::ForceSensor")
      force = depl:getPeer("force")
      force:getProperty("cuttoff_freq"):set(1)
      force:getProperty("sample_rate"):set(1/timefreq)
      force:getProperty("weight_increment"):set(0.004)
      force:getProperty("force_threshold"):set(0.5)
      force:getProperty("torque_threshold"):set(0.03)
      -- Tool Changers
      -- force:getProperty("tool_center_gravity"):set(d{-0.00032338848790914644, -0.00418111, -0.1003717027520959})
      -- force:getProperty("tool_weight_vector"):set(d{0.0, 0.0, -19.236133805938685})
      --force:getProperty("tool_center_gravity"):set(d{0.004726095658581488, 0.001106237582689138, -0.07333410225335406})
      --force:getProperty("tool_weight_vector"):set(d{0.0, 0.0, -12.021139670520544})

      --For planar contour following tool;
      -- LONG ROD
      --force:getProperty("tool_center_gravity"):set(d{-0.000675944, 0.00110449, -0.111525})
      --force:getProperty("tool_weight_vector"):set(d{0.0, 0.0, -2.32884})

      --SHORT ROD
      force:getProperty("tool_center_gravity"):set(d{0.00148262, 0.000583076, -0.0517289})
      force:getProperty("tool_weight_vector"):set(d{0.0, 0.0, -2.02485})

      -- force:getProperty("tool_center_gravity"):set(du.ftab2arr({0.0007,-0.0257366,-0.0156536}))
      -- force:getProperty("tool_weight_vector"):set(du.ftab2arr({0.0,0.0,-2.11676}))
      depl:setActivity("force", 0, 0, rtt.globals.ORO_SCHED_RT)

      -- deploy tf broadcaster:
      ros:import("rtt_tf")
      depl:loadComponent("tf","rtt_tf::RTT_TF")
      depl:connectServices("force","tf")
      tf = depl:getPeer("tf")
      tf:configure()
      tf:start()

      -- depl:stream("force.wrench", ros:topic("/wrench_raw"))
      force:configure()
      force:start()
      jr3:provides("sensor_0"):resetForces()
      -- depl:connectPeers("supervisor","jr3")

-- ====================================== Configure eTaSL ports of F/T sensor
--     wr=s{"global.Fx","global.Fy","global.Fz","global.Tx","global.Ty","global.Tz"}
--     F_w=s{"global.W_Fx","global.W_Fy","global.W_Fz","global.W_Tx","global.W_Ty","global.W_Tz"}
--     comp:add_etaslvar_inputport("wrench_array","Vector with wrench values",wr,d{})
--     comp:add_etaslvar_inputport("force_weights","Vector with constraint weights",F_w,d{})
--     comp:add_etaslvar_deriv_inputport("force_weights_derivatives", "Vector with constraint weights derivatives", F_w)
    comp:add_etaslvar_frame_outputport("FT_frame","Frame of the load cell for the force component","FT_frame")

      -- Force ports
      depl:connect("jr3.sensor_0.wrench_filter_0","force.wrench",cp )
      depl:connect(compname..".FT_frame","force.FT_frame",cp )
      depl:connect("force.wrench_array",compname..".wrench_array",cp )
      -- depl:connect("force.force_weights",compname..".force_weights",cp )
      -- depl:connect("force.force_weights_derivatives",compname..".force_weights_derivatives",cp )
end


-- export functions
M.FTsensor_deployer = FTsensor_deployer

return M
