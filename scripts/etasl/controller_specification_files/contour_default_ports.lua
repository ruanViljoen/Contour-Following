-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Main file to define default robot and ports
-- KU Leuven 2020
-- ==============================================================================
require("context")
require("geometric")
-- robot.etasl_specification()

-- ==================== Input Ports =====================================
dot_xu_tracking = ctx:createInputChannelScalar("dot_xu_tracking")
dot_xu_force = ctx:createInputChannelScalar("dot_xu_force")

xu_tracking = ctx:createInputChannelScalar("xu_tracking")
xu_force = ctx:createInputChannelScalar("xu_force")

xu_x = ctx:createInputChannelScalar("xu_x")
xu_y = ctx:createInputChannelScalar("xu_y")

dot_x_ee = ctx:createInputChannelScalar("dot_x_ee")

vx_d = ctx:createInputChannelScalar("vx_d")
vy_d = ctx:createInputChannelScalar("vy_d")

Fx_raw = ctx:createInputChannelScalar("Fx")
Fy_raw = ctx:createInputChannelScalar("Fy")
Fz_raw = ctx:createInputChannelScalar("Fz")
Tx_raw = ctx:createInputChannelScalar("Tx")
Ty_raw = ctx:createInputChannelScalar("Ty")
Tz_raw = ctx:createInputChannelScalar("Tz")

max_joint_vels ={}
min_joint_vels ={}
current_joint_vels ={}
for i=1,#robot_joints do
  max_joint_vels[i] = ctx:createInputChannelScalar("maxvel_j"..i)
  min_joint_vels[i] = ctx:createInputChannelScalar("minvel_j"..i)
  current_joint_vels[i] = ctx:createInputChannelScalar("curr_vel_j"..i)
end

-- ==================== Output ports ==================================

ctx:setOutputExpression("FT_frame",FT_sensor_frame)

ctx:setOutputExpression("x_tf",constant(0))
ctx:setOutputExpression("y_tf",constant(0))
ctx:setOutputExpression("z_tf",constant(0))

ctx:setOutputExpression("roll_tf",constant(0))
ctx:setOutputExpression("pitch_tf",constant(0))
ctx:setOutputExpression("yaw_tf",constant(0))

ctx:setOutputExpression("x_TCP",constant(0))
ctx:setOutputExpression("y_TCP",constant(0))
ctx:setOutputExpression("z_TCP",constant(0))

ctx:setOutputExpression("roll_TCP",constant(0))
ctx:setOutputExpression("pitch_TCP",constant(0))
ctx:setOutputExpression("yaw_TCP",constant(0))

ctx:setOutputExpression("x_fs",constant(0))
ctx:setOutputExpression("y_fs",constant(0))
ctx:setOutputExpression("z_fs",constant(0))

ctx:setOutputExpression("roll_fs",constant(0))
ctx:setOutputExpression("pitch_fs",constant(0))
ctx:setOutputExpression("yaw_fs",constant(0))

ctx:setOutputExpression("Fx_out",constant(0))
ctx:setOutputExpression("Fy_out",constant(0))
ctx:setOutputExpression("Fz_out",constant(0))

ctx:setOutputExpression("Tx_out",constant(0))
ctx:setOutputExpression("Ty_out",constant(0))
ctx:setOutputExpression("Tz_out",constant(0))

ctx:setOutputExpression("Fx_sensor",constant(0))
ctx:setOutputExpression("Fy_sensor",constant(0))
ctx:setOutputExpression("Fz_sensor",constant(0))

ctx:setOutputExpression("Tx_sensor",constant(0))
ctx:setOutputExpression("Ty_sensor",constant(0))
ctx:setOutputExpression("Tz_sensor",constant(0))

ctx:setOutputExpression("y_ICTF", constant(0))
ctx:setOutputExpression("delta", constant(0))
ctx:setOutputExpression("xf", constant(0))

ctx:setOutputExpression("xu_normal", constant(0))
ctx:setOutputExpression("xu_theta", constant(0))