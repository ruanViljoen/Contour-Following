
-- ==============================================================================
-- Author: Ruan Viljoen (adapted from the code of Rob Eyskens)
-- ==============================================================================

require("context")
require("geometric")
utils_ts = require("utils_ts")
require("math")

-- ======================================== FRAMES ========================================

-- Additional variables
xu_theta = Variable {
    context = ctx,
    name = 'xu_theta',
    vartype = "feature",
    initial = 1.57
}

xu_normal = Variable {
    context = ctx,
    name = 'xu_normal',
    vartype = "feature"
}

x_v2 = Variable {
    context = ctx,
    name = 'x_v2',
    vartype = "feature"
}

x_v1 = Variable {
    context = ctx,
    name = 'x_v1',
    vartype = "feature",
    initial = 1.57,
    weight = 0.01
}

R = rotate_z(x_v1)
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

ctx:setOutputExpression("Fx_sensor", Fx_raw)
ctx:setOutputExpression("Fy_sensor", Fy_raw)
ctx:setOutputExpression("Fz_sensor", Fz_raw)

F_sensor = vector(Fx_raw, Fy_raw, Fz_raw)

R_sensor_tcp = rotation(T_sensor_tcp) -- from URDF file, defined in "ur10_etasl.lua"
R_tcp_tf = rot_z(make_constant(x_v1))

F_tf = inv(R_tcp_tf) * inv(R_sensor_tcp) * F_sensor

Fx_tf = coord_x(F_tf)
Fy_tf = coord_y(F_tf)
Fz_tf = coord_z(F_tf)

ctx:setOutputExpression("Fx_out", Fx_tf)
ctx:setOutputExpression("Fy_out", Fy_tf)
ctx:setOutputExpression("Fz_out", Fz_tf)

y_err = conditional(norm(F_sensor)-constant(1), atan2(Fy_tf , Fx_tf), constant(0))


-- ======================================== ICTF ========================================
-- used to express the velocity of the task frame with respect to itself
tf_i = inv(make_constant(tf)) * tf
pos_vec = origin(tf_i)
rot_vec = getRotVec(rotation(tf_i))

-- ======================================================================================
-- Constraint specification  ============================================================
-- ======================================================================================

-- ======================================== ESTIMATOR ===================================
est_R_w_xu = rot_z(xu_theta)
est_R_xu_tf = inv(est_R_w_xu) * rotation(make_constant(tf))

Constraint {
    context = ctx,
    name = "Rotation around z",
    expr = - coord_z(getRotVec(est_R_xu_tf)),
    target = y_err,
    weight = 1,
    K = 3,
    priority = 1
}

-- Integrate "x_v2"
Constraint {
    context = ctx,
    name = "x_v2",
    expr = x_v2,
    target = coord_x(pos_vec),
    weight = 1,
    K = 0,
    priority = 1
}

-- Estimate nominal surface height
Constraint {
    context = ctx,
    name = "xu_normal",
    expr = estimated_stiffness*(xu_normal - make_constant(x_v2)),
    target = Fx_tf,
    weight = 1,
    K = 3,
    priority = 1
}

-- ======================================== CONTROLLER ========================================
-- Constraint to keep TCP (roller) at a fixed height
Constraint {
    context = ctx,
    name = "z",
    expr = tgt_z - coord_z(origin(TCP)),
    weight = 1,
    K = 3,
    priority = 1
}

-- Conditional statement to turn on disturbance rejection after 5 seconds
R_w_xu = rot_z(conditional(time - constant(5), xu_theta, make_constant(xu_theta)))
R_xu_tf = inv(R_w_xu) * rotation(tf)

-- Constraint to keep roller plane parallel to surface (1)
Constraint {
    context = ctx,
    name = "Orientation_x",
    expr = coord_x(getRotVec(R_xu_tf)),
    target = 0,
    K = 3,
    weight = 1,
    priority = 1
}

-- Constraint to keep roller plane parallel to surface (2)
Constraint {
    context = ctx,
    name = "Orientation_y",
    expr = coord_y(getRotVec(R_xu_tf)),
    target = 0,
    K = 3,
    weight = 1,
    priority = 1
}

-- Orientation constraint to align tf with surface normal
Constraint {
    context = ctx,
    name = "Orientation_z",
    model = - coord_z(getRotVec(R_xu_tf)),
    meas = y_err,
    target = 0,
    K = 3,
    weight = 1,
    priority = 1
}

-- Force constraint in normal direction
Constraint {
    context = ctx,
    name = "Normal force",
    model = -(make_constant(x_v2) + coord_x(pos_vec)) + conditional(time - constant(5), xu_normal, make_constant(xu_normal)),
    meas = estimated_compliance * Fx_tf,
    target = estimated_compliance * F_des,
    weight = 1,
    K = 5,
    priority = 1
}


-- Tangential velocity constraint 
-- Only start after 5 seconds
v_des_delayed = conditional(time - constant(5), v_des, constant(0))
Constraint {
    context = ctx,
    name = "Tangential velocity",
    expr = coord_y(pos_vec),
    target = v_des_delayed*time,
    weight = 1,
    K = 0,
    priority = 1
}

-- =========================== MONITORS ============================================

-- Monitor end task after certain time duration
duration = 120
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

ctx:setOutputExpression("xu_normal", xu_normal)
ctx:setOutputExpression("xu_theta", xu_theta)