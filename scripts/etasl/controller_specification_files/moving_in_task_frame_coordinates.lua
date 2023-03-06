 -- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Main file to move linearly along a frame axis with trapezoidal velocity
-- profile
-- KU Leuven 2020
-- ==============================================================================

require("context")
require("geometric")
utils_ts = require("utils_ts")

-- ========================================= PARAMETERS ===================================
maxvel    = ctx:createInputChannelScalar("maxvel" ,0.1)
maxacc    = ctx:createInputChannelScalar("maxacc" ,0.1)
eqradius  = ctx:createInputChannelScalar("eq_r"   ,0.08)
delta_x   = ctx:createInputChannelScalar("delta_x",0.0)
delta_y   = ctx:createInputChannelScalar("delta_y",0.0)
delta_z   = ctx:createInputChannelScalar("delta_z",0.0)

-- ======================================== FRAMES ========================================

tf = tcp_frame

-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, tf)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- =============================== END POSE ==============================
T_a_b = translate_x(delta_x)*translate_y(delta_y)*translate_z(delta_z) -- *rotate_z(constant(3.14159/4))
endpose   = startpose*T_a_b
-- endpose   = startpose*translate_x(delta_x)*translate_y(delta_y)*translate_z(delta_z)
endpos    = origin(endpose)
endrot    = rotation(endpose)

-- =========================== VELOCITY PROFILE ============================================

-- compute distances for displacements and rotations:
diff                    = cached(endpos-startpos)
diff, distance          = utils_ts.normalize( diff )

diff_rot                = cached(  getRotVec( inv(startrot)*endrot )) -- eq. axis of rotation for rotation from start to end:w
diff_rot, angle         = utils_ts.normalize( diff_rot )


-- plan trapezoidal motion profile in function of time:
mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
mp:addOutput(constant(0), distance, maxvel, maxacc)
mp:addOutput(constant(0), angle*eqradius, maxvel, maxacc)
d  = get_output_profile(mp,0)            -- progression in distance
r  = get_output_profile(mp,1)/eqradius   -- progression in distance_rot (i.e. rot*eqradius)

-- =========================== TARGET POSE ============================================

targetpos = startpos + diff*d
targetrot = startrot*rotVec(diff_rot,r)

target    = frame(targetrot,targetpos)

-- ========================== CONSTRAINT SPECIFICATION =================================
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*tf,
    K       = 3,
    weight  = 1,
    priority= 2
}

-- =========================== MONITOR ============================================
Monitor{
        context=ctx,
        name='finish_after_motion',
        upper=0.0,
        actionname='exit',
        expr=time-get_duration(mp) - constant(0.1)
}


-- ============================== OUTPUT THROUGH PORTS===================================
ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
ctx:setOutputExpression("z_tf",coord_z(origin(tf)))

roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)

TCP = tcp_frame
ctx:setOutputExpression("x_TCP", coord_x(origin(TCP)))
ctx:setOutputExpression("y_TCP", coord_y(origin(TCP)))
ctx:setOutputExpression("z_TCP", coord_z(origin(TCP)))

roll_TCP, pitch_TCP, yaw_TCP = getRPY(rotation(TCP))
ctx:setOutputExpression("roll_TCP", roll_TCP)
ctx:setOutputExpression("pitch_TCP", pitch_TCP)
ctx:setOutputExpression("yaw_TCP", yaw_TCP)