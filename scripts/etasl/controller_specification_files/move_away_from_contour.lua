
-- ==============================================================================
-- Author: Ruan Viljoen (adapted from the code of Rob Eyskens)
-- ==============================================================================

require("context")
require("geometric")
utils_ts = require("utils_ts")
require("math")

-- ======================================== FRAMES ========================================

xf = Variable {
    context = ctx,
    name = 'xf',
    vartype = "feature",
    weight = 0.01
}
R = rotate_z(xf)
tf = tcp_frame * R

-- ============================== OUTPUT THROUGH PORTS===================================
TCP = tcp_frame
ctx:setOutputExpression("x_TCP", coord_x(origin(TCP)))
ctx:setOutputExpression("y_TCP", coord_y(origin(TCP)))
ctx:setOutputExpression("z_TCP", coord_z(origin(TCP)))

roll_TCP, pitch_TCP, yaw_TCP = getRPY(rotation(TCP))
ctx:setOutputExpression("roll_TCP", roll_TCP)
ctx:setOutputExpression("pitch_TCP", pitch_TCP)
ctx:setOutputExpression("yaw_TCP", yaw_TCP)

ctx:setOutputExpression("x_tf", coord_x(origin(tf)))
ctx:setOutputExpression("y_tf", coord_y(origin(tf)))
ctx:setOutputExpression("z_tf", coord_z(origin(tf)))

roll_tf, pitch_tf, yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf", roll_tf)
ctx:setOutputExpression("pitch_tf", pitch_tf)
ctx:setOutputExpression("yaw_tf", yaw_tf)

fs = FT_sensor_frame
ctx:setOutputExpression("x_fs", coord_x(origin(fs)))
ctx:setOutputExpression("y_fs", coord_y(origin(fs)))
ctx:setOutputExpression("z_fs", coord_z(origin(fs)))

roll_fs, pitch_fs, yaw_fs = getRPY(rotation(fs))
ctx:setOutputExpression("roll_fs", roll_fs)
ctx:setOutputExpression("pitch_fs", pitch_fs)
ctx:setOutputExpression("yaw_fs", yaw_fs)

-- ========================================= PARAMETERS ===================================
F_des = ctx:createInputChannelScalar("F_des")
v_des = ctx:createInputChannelScalar("v_des")
estimated_stiffness = ctx:createInputChannelScalar("estimated_stiffness")
estimated_compliance = 1 / estimated_stiffness
tgt_z = ctx:createInputChannelScalar("tgt_z")

-- ========================================= FORCES ============

-- The goal is to transform the forces from the force sensor frame to the task frame. 

F_sensor = vector(Fx_raw, Fy_raw, Fz_raw)

R_sensor_tcp = rotation(T_sensor_tcp) -- from URDF file, defined in "ur10_etasl.lua"
R_tcp_tf = rot_z(make_constant(xf))

F_tf = inv(R_tcp_tf) * inv(R_sensor_tcp) * F_sensor

Fx_tf = coord_x(F_tf)
Fy_tf = coord_y(F_tf)
Fz_tf = coord_z(F_tf)

-- ======================================== ICTF ========================================

tf_i = inv(make_constant(tf)) * tf
pos_vec = origin(tf_i)
rot_vec = getRotVec(rotation(tf_i))





-- ==============================================================================================================
-- Constraint specification  ====================================================================================
-- ==============================================================================================================

-- Height of TCP
Constraint {
    context = ctx,
    name = "z",
    expr = tgt_z - coord_z(origin(TCP)),
    weight = 1,
    K = 3,
    priority = 2
}

-- Constant roll
Constraint {
    context = ctx,
    name = "Rotation around x",
    expr = roll_tf,
    weight = 1,
    K = 3,
    priority = 2
}

-- Constant pitch
Constraint {
    context = ctx,
    name = "Rotation around y",
    expr = pitch_tf,
    weight = 1,
    K = 3,
    priority = 2
}

-- Tracking
delta = conditional(norm(F_sensor) - constant(1), atan2(Fx_tf, Fy_tf), constant(0)) -- Calculate error only if force magnitude is larger than 1N, otherwise use error = 0
Constraint {
    context = ctx,
    name = "Task frame tracking",
    model = coord_z(rot_vec),
    meas = delta,
    target = 0,
    weight = 1,
    K = 2,
    priority = 2
}

-- Normal velocity
Constraint {
    context = ctx,
    name = "Normal velocity",
    expr = coord_y(pos_vec),
    target = conditional(time - constant(5), 0.005 * time, constant(0)),
    weight = 1,
    K = 0,
    priority = 2
}

-- Tangential velocity
Constraint {
    context = ctx,
    name = "Tangential velocity",
    expr = coord_x(pos_vec),
    target = 0,
    weight = 1,
    K = 0,
    priority = 2
}





-- =========================== MONITORS ============================================

-- Monitor end task after certain time duration
duration = 10
Monitor {
    context = ctx,
    name = 'Contour following finished',
    upper = 0.0,
    actionname = 'exit',
    expr = time - duration
}

-- Monitor if large forces appeares
F_threshold = 50
Monitor {
    context = ctx,
    name = 'Finished due to high contact forces',
    upper = 0.0,
    actionname = 'exit',
    expr = norm(F_tf) - F_threshold
}

-- ============================== OUTPUT THROUGH PORTS===================================
ctx:setOutputExpression("Fx_out", Fx_tf)
ctx:setOutputExpression("Fy_out", Fy_tf)
ctx:setOutputExpression("Fz_out", Fz_tf)

ctx:setOutputExpression("Fx_sensor", Fx_raw)
ctx:setOutputExpression("Fy_sensor", Fy_raw)
ctx:setOutputExpression("Fz_sensor", Fz_raw)

-- ctx:setOutputExpression("y_ICTF", coord_y(pos_vec))
-- ctx:setOutputExpression("delta", delta)
-- ctx:setOutputExpression("xf", xf)