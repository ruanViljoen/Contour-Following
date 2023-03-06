-- Example of trapezoidal motionprofile, applied to a joint space movement:
-- Requires the following etal variables to be set:
--     maxvel
--     maxacc
--     end_1 ... end_n (jval at the end of the motion)


require("context")
require("geometric")
utils_ts = require("utils_ts")

-- ========================================= PARAMETERS ===================================
maxvel = ctx:createInputChannelScalar("maxvel",0.5)
maxacc = ctx:createInputChannelScalar("maxacc",0.5)

-- ========================================= VELOCITY PROFILE ===================================

mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
current_jnt = {} -- current joint value
for i=1,#robot_joints do
    current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
    mp:addOutput( initial_value(time, current_jnt[i]), ctx:createInputChannelScalar("end_j"..i), maxvel, maxacc)
end
duration = get_duration(mp)

-- ========================= CONSTRAINT SPECIFICATION ========================

tgt         = {} -- target value
for i=1,#robot_joints do
    tgt[i]        = get_output_profile(mp,i-1)
    Constraint{
        context=ctx,
        name="joint_trajectory"..i,
        expr= current_jnt[i] - tgt[i] ,
        K=5
    };
end

-- =================================== MONITOR TO FINISH THE MOTION ========================

Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-(duration)
}


-- ============================== GENERATE FORCE OUTPUTS ===================================

Fx_raw = ctx:createInputChannelScalar("Fx")
Fy_raw = ctx:createInputChannelScalar("Fy")
Fz_raw = ctx:createInputChannelScalar("Fz")
Tx_raw = ctx:createInputChannelScalar("Tx")
Ty_raw = ctx:createInputChannelScalar("Ty")
Tz_raw = ctx:createInputChannelScalar("Tz")

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
-- R_tcp_tf =  rot_z(xu)
R_tcp_tf =  rot_z(constant(0))
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

F_vector = vector(Fx,Fy,Fz)

-- ============================== OUTPUT THROUGH PORTS===================================
TCP = tcp_frame

fs = FT_sensor_frame

ctx:setOutputExpression("x_TCP",coord_x(origin(TCP)))
ctx:setOutputExpression("y_TCP",coord_y(origin(TCP)))
ctx:setOutputExpression("z_TCP",coord_z(origin(TCP)))

roll_TCP,pitch_TCP,yaw_TCP = getRPY(rotation(TCP))
ctx:setOutputExpression("roll_TCP",roll_TCP)
ctx:setOutputExpression("pitch_TCP",pitch_TCP)
ctx:setOutputExpression("yaw_TCP",yaw_TCP)

ctx:setOutputExpression("x_fs",coord_x(origin(fs)))
ctx:setOutputExpression("y_fs",coord_y(origin(fs)))
ctx:setOutputExpression("z_fs",coord_z(origin(fs)))

roll_fs,pitch_fs,yaw_fs = getRPY(rotation(fs))
ctx:setOutputExpression("roll_fs",roll_fs)
ctx:setOutputExpression("pitch_fs",pitch_fs)
ctx:setOutputExpression("yaw_fs",yaw_fs)

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