-- ==============================================================================
-- Author: Rob Eyskens
-- 2D contour following using force feedback - move towards object
-- ==============================================================================


require("context")
require("geometric")
utils_ts = require("utils_ts")

-- Above in file because the force component is not able to calculate simulation force otherwise.
-- ======================================== FRAMES ========================================

TCP = tcp_frame
task_frame = tcp_frame

-- ============================== OUTPUT THROUGH PORTS===================================
-- Used by force simulation to determine force
ctx:setOutputExpression("x_TCP",coord_x(origin(TCP)))
ctx:setOutputExpression("y_TCP",coord_y(origin(TCP)))
ctx:setOutputExpression("z_TCP",coord_z(origin(TCP)))

roll_TCP,pitch_TCP,yaw_TCP = getRPY(rotation(TCP))
ctx:setOutputExpression("roll_TCP",roll_TCP)
ctx:setOutputExpression("pitch_TCP",pitch_TCP)
ctx:setOutputExpression("yaw_TCP",yaw_TCP)

fs = FT_sensor_frame

ctx:setOutputExpression("x_fs",coord_x(origin(fs)))
ctx:setOutputExpression("y_fs",coord_y(origin(fs)))
ctx:setOutputExpression("z_fs",coord_z(origin(fs)))

roll_fs,pitch_fs,yaw_fs = getRPY(rotation(fs))
ctx:setOutputExpression("roll_fs",roll_fs)
ctx:setOutputExpression("pitch_fs",pitch_fs)
ctx:setOutputExpression("yaw_fs",yaw_fs)


-- ========================================= PARAMETERS ===================================
Fx_raw = ctx:createInputChannelScalar("Fx")
Fy_raw = ctx:createInputChannelScalar("Fy")
Fz_raw = ctx:createInputChannelScalar("Fz")
Tx_raw = ctx:createInputChannelScalar("Tx")
Ty_raw = ctx:createInputChannelScalar("Ty")
Tz_raw = ctx:createInputChannelScalar("Tz")

F_des = ctx:createInputChannelScalar("F_des")
v_des = ctx:createInputChannelScalar("v_des")
tgt_x = ctx:createInputChannelScalar("tgt_x")
tgt_z = ctx:createInputChannelScalar("tgt_z")
th_f = ctx:createInputChannelScalar("th_f")
th_t = ctx:createInputChannelScalar("th_t")

Fx_lc  = utils_ts.dead_zone(Fx_raw,th_f)
Fy_lc  = utils_ts.dead_zone(Fy_raw,th_f)
Fz_lc  = utils_ts.dead_zone(Fz_raw,th_f)
Tx_lc  = utils_ts.dead_zone(Tx_raw,th_t)
Ty_lc  = utils_ts.dead_zone(Ty_raw,th_t)
Tz_lc  = utils_ts.dead_zone(Tz_raw,th_t)

wr_lc = wrench(vector(Fx_lc,Fy_lc,Fz_lc),vector(Tx_lc,Ty_lc,Tz_lc))

R_sensor_tcp = rotation(T_sensor_tcp)
R_tcp_tf =  rot_z(xu)
R_sensor_tf = R_sensor_tcp * R_tcp_tf
R_tf_sensor = inv(R_sensor_tf)
wr_tf1 = transform(R_tf_sensor, wr_lc)
wr_tf = ref_point(wr_tf1, origin(T_sensor_tcp))

Fx = coord_x(force(wr_tf))
Fy = coord_y(force(wr_tf))
Fz = coord_z(force(wr_tf))
Tx = coord_x(torque(wr_tf))
Ty = coord_y(torque(wr_tf))
Tz = coord_z(torque(wr_tf))

F_vector = vector(Fx,Fy,Fz) -- Measured forces in frame of TCP

task_frame_i = inv(make_constant(task_frame))*task_frame
-- pos_vec = origin(task_frame_i)
-- rot_vec = getRotVec(rotation(task_frame_i))


-- ========================== CONSTRAINT SPECIFICATION =================================
-- Constraint{
--     context = ctx,
--     name    = "direction perpendicular to dual robot setup",
--     expr    = v_des * time - coord_x(pos_vec),
--     K       = 0,
--     weight  = 1
-- }

-- Constraint{
--     context = ctx,
--     name    = "direction parallel to dual robot setup",
--     expr    = tgt_x - coord_x(origin(TCP)),
--     K       = 1,
--     weight  = 1
-- }

-- Constraint{
--     context = ctx,
--     name    = "z direction",
--     expr    = tgt_z - coord_z(origin(TCP)),
--     K       = 1,
--     weight  = 1
-- }

Constraint{
    context = ctx,
    name    = "x",
    expr    = coord_x(origin(task_frame_i)),
    target  = 0,
    K       = 0,
    weight  = 1
}

Constraint{
    context = ctx,
    name    = "y",
    expr    = coord_y(origin(task_frame_i)),
    target  = -v_des * time,
    K       = 0,
    weight  = 1
}

-- Constraint{
--     context = ctx,
--     name    = "z",
--     expr    = coord_z(origin(task_frame)),
--     target  = 0,
--     K       = 0,
--     weight  = 1
-- }

-- Height of TCP
Constraint {
    context = ctx,
    name = "z",
    expr = tgt_z - coord_z(origin(TCP)),
    weight = 1,
    K = 3,
    priority = 2
}

targetrot = rot_z(constant(3.14159/2))*rot_y(constant(0))*rot_x(constant(0))
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(make_constant(targetrot))*rotation(task_frame),
    K       = 1,
    weight  = 1
}


-- =========================== MONITOR ============================================
Monitor{
        context=ctx,
        name='contact detected',
        upper= 0.0,
        actionname='exit',
        expr= Fy - F_des -- Measured force in opposite direction to  
}

-- ============================== OUTPUT THROUGH PORTS===================================

ctx:setOutputExpression("Fx_out",Fx)
ctx:setOutputExpression("Fy_out",Fy)
ctx:setOutputExpression("Fz_out",Fz)
ctx:setOutputExpression("Tx_out",Tx)
ctx:setOutputExpression("Ty_out",Ty)
ctx:setOutputExpression("Tz_out",Tz)

ctx:setOutputExpression("Fx_sensor", Fx_lc)
ctx:setOutputExpression("Fy_sensor", Fy_lc)
ctx:setOutputExpression("Fz_sensor", Fz_lc)
ctx:setOutputExpression("Tx_sensor", Tx_lc)
ctx:setOutputExpression("Ty_sensor", Ty_lc)
ctx:setOutputExpression("Tz_sensor", Tz_lc)