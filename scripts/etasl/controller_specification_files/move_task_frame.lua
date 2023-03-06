-- ==============================================================================
-- Author: Cristian Vergara and Federico Ulloa
-- email: <cristian.vergara@kuleuven.be>
-- Task specification to use admittance
-- KU Leuven 2021
-- ==============================================================================
require("context")
require("geometric")
require("libexpressiongraph_velocities")
utils_ts = require("utils_ts")

-- ============================= CHECK AND INITIALIZE VARIABLES ========================================
if not task_frame then
	print("Please define a task frame")
end

if not FT_sensor_frame then
	print("Please define the frame of the FT sensor")
end

if not feature then
	feature = {}
end

feature[feature_i] = {}
-- ========================================= PARAMETERS ===================================

-- Magnitud
tgt_x   = ctx:createInputChannelScalar("tgt_x"  ,0.0)
tgt_y   = ctx:createInputChannelScalar("tgt_y"  ,0.0)
tgt_z   = ctx:createInputChannelScalar("tgt_z"  ,0.0)
tgt_r_x = ctx:createInputChannelScalar("tgt_r_x"  ,0.0)
tgt_r_y = ctx:createInputChannelScalar("tgt_r_y"  ,0.0)
tgt_r_z   = ctx:createInputChannelScalar("tgt_r_z"  ,0.0)

x_mode  = ctx:createInputChannelScalar("x_mode"  ,1.0)
y_mode  = ctx:createInputChannelScalar("y_mode"  ,1.0)
z_mode  = ctx:createInputChannelScalar("z_mode"  ,1.0)
rx_mode  = ctx:createInputChannelScalar("rx_mode"  ,1.0)
ry_mode  = ctx:createInputChannelScalar("ry_mode"  ,1.0)
rz_mode  = ctx:createInputChannelScalar("rz_mode"  ,1.0)

K_Fx = ctx:createInputChannelScalar("K_Fx"  ,2700) -- compliance in x-axis
K_Fy = ctx:createInputChannelScalar("K_Fy"  ,2700) -- compliance in y-axis
K_Fz = ctx:createInputChannelScalar("K_Fz"  ,2700) -- compliance in z-axis
K_Tx = ctx:createInputChannelScalar("K_Tx"  ,200) -- compliance about x-axis
K_Ty = ctx:createInputChannelScalar("K_Ty"  ,200) -- compliance about y-axis
K_Tz = ctx:createInputChannelScalar("K_Tz"  ,200) -- compliance about z-axis

-- C_Ty = constant(0.25) -- compliance about y-axis

-- insertion_vector = ctx:createInputChannelVector("insertion_vector")
tcp_2_tf =  ctx:createInputChannelFrame("tcp_2_tf")

-- TODO move taskframe to peg/hole location
feature[feature_i]['tf'] = cached(task_frame*tcp_2_tf)

force_threshold = ctx:createInputChannelScalar("force_threshold"  , 0.5)
torque_threshold = ctx:createInputChannelScalar("torque_threshold"  , 0.03)

-- ==================================== SIGNALS PRE-PROCESSING ===========================

Fx_after_dz  = utils_ts.dead_zone(Fx_raw,force_threshold)
Fy_after_dz  = utils_ts.dead_zone(Fy_raw,force_threshold)
Fz_after_dz  = utils_ts.dead_zone(Fz_raw,force_threshold)
Tx_after_dz  = utils_ts.dead_zone(Tx_raw,torque_threshold)
Ty_after_dz  = utils_ts.dead_zone(Ty_raw,torque_threshold)
Tz_after_dz  = utils_ts.dead_zone(Tz_raw,torque_threshold)

wr_after_dz = wrench(vector(Fx_after_dz,Fy_after_dz,Fz_after_dz),vector(Tx_after_dz,Ty_after_dz,Tz_after_dz))

wr_tf   = ref_point(transform(rotation( inv(feature[feature_i]['tf'])*FT_sensor_frame ),wr_after_dz) , -origin(inv(feature[feature_i]['tf'])*FT_sensor_frame))

Fx = coord_x(force(wr_tf))
Fy = coord_y(force(wr_tf))
Fz = coord_z(force(wr_tf))
Tx = coord_x(torque(wr_tf))
Ty = coord_y(torque(wr_tf))
Tz = coord_z(torque(wr_tf))

-- =============================== INSTANTANEOUS FRAME ==============================

feature[feature_i]['task_frame_i'] = inv(make_constant(feature[feature_i]['tf']))*feature[feature_i]['tf']

-- =============================== VARIABLES SPECIFICATION ===============================
-- -- Traveled distance
-- traveled_distance_z = Variable{context=ctx, name='traveled_distance_z'..feature_i..'', vartype ='feature', initial = 0}
--
-- velocity_y = Variable{context=ctx, name='velocity_y'..feature_i..'', vartype ='feature', initial = 0.01}
--
-- velocity_z = Variable{context=ctx, name='velocity_z'..feature_i..'', vartype ='feature', initial = 0.01}
--
--
-- -- =============================== INTEGRATOR TO COMPUTE DISTANCE ===============================
-- velocity_z = previous_velocity(time, origin(task_frame))
-- Constraint{
-- 	context=ctx,
-- 	name="traveled_distance"..feature_i.."",
-- 	expr=coord_z(origin(feature[feature_i]['task_frame_i'])) - traveled_distance_z,
-- 	K = 0,
-- 	priority = 2,
-- 	weight = constant(1),
-- };
--
-- Constraint{
-- 	context=ctx,
-- 	name="velocity_y_calc"..feature_i.."",
-- 	expr=coord_y(origin(feature[feature_i]['task_frame_i'])) - velocity_y*time,
-- 	K = 0,
-- 	priority = 2,
-- 	weight = constant(1),
-- };
--
-- Constraint{
-- 	context=ctx,
-- 	name="velocity_z_calc"..feature_i.."",
-- 	expr=coord_z(origin(feature[feature_i]['task_frame_i'])) - velocity_z*time,
-- 	K = 0,
-- 	priority = 2,
-- 	weight = constant(1),
-- };

-- =============================== CONSTRAINT SPECIFICATION ==============================

---- Force constraints
Constraint{
	context=ctx,
	name="follow_force_x_"..feature_i.."",
	model = -K_Fx*coord_x(origin(feature[feature_i]['task_frame_i'])),
	meas = Fx,
	target = tgt_x,
	K = constant(4),
	priority = 2,
	weight = constant(1) - x_mode,
};

Constraint{
	context=ctx,
	name="follow_force_y_"..feature_i.."",
	model = -K_Fy*coord_y(origin(feature[feature_i]['task_frame_i'])),
	meas = Fy,
	target = tgt_y,
	K = constant(4),
	priority = 2,
	weight = constant(1) - y_mode,
};

Constraint{
	context=ctx,
	name="follow_force_z_"..feature_i.."",
	model = -K_Fz*coord_z(origin(feature[feature_i]['task_frame_i'])),
	meas = Fz,
	target = tgt_z,
	K = constant(4),
	priority = 2,
	weight = constant(1) - z_mode,
};

-- Translation velocities
Constraint{
    context = ctx,
    name    = "x_velocity_"..feature_i.."",
    expr    = coord_x( origin(feature[feature_i]['task_frame_i'])) - tgt_x*time,
    K       = 0,
    weight  = x_mode,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "y_velocity_"..feature_i.."",
    expr    = coord_y( origin(feature[feature_i]['task_frame_i'])) - tgt_y*time,
    K       = 0,
    weight  = y_mode,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "z_velocity_"..feature_i.."",
    expr    = coord_z( origin(feature[feature_i]['task_frame_i'])) - tgt_z*time,
    K       = 0,
    weight  = z_mode,
    priority= 2
};

---- Rotation torque constraints
Constraint{
	context=ctx,
	name="follow_torque_x_"..feature_i.."",
	model = -K_Tx*coord_x(getRotVec(rotation(feature[feature_i]['task_frame_i']))),
	meas = Tx,
	target = tgt_r_x,
	K = constant(4),
	priority = 2,
	weight = constant(1) - rx_mode,
};

Constraint{
	context=ctx,
	name="follow_torque_y_"..feature_i.."",
	model = -K_Ty*coord_y(getRotVec(rotation(feature[feature_i]['task_frame_i']))),
	meas = Ty,
	target = tgt_r_y,
	K = constant(4),
	priority = 2,
	weight = constant(1) - ry_mode,
};

Constraint{
	context=ctx,
	name="follow_torque_z_"..feature_i.."",
	model = -K_Tz*coord_z(getRotVec(rotation(feature[feature_i]['task_frame_i']))),
	meas = Tz,
	target = tgt_r_z,
	K = constant(4),
	priority = 2,
	weight = constant(1) - rz_mode,
};

-- Translation velocities
Constraint{
    context = ctx,
    name    = "x_angular_"..feature_i.."",
    expr    = coord_x(getRotVec(rotation(feature[feature_i]['task_frame_i']))) - tgt_r_x*time,
    K       = 0,
    weight  = rx_mode,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "y_angular_"..feature_i.."",
    expr    = coord_y(getRotVec(rotation(feature[feature_i]['task_frame_i']))) - tgt_r_y*time,
    K       = 0,
    weight  = ry_mode,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "z_angular_"..feature_i.."",
    expr    = coord_z(getRotVec(rotation(feature[feature_i]['task_frame_i']))) - tgt_r_z*time,
    K       = 0,
    weight  = rz_mode,
    priority= 2
};

-- Rotation angular velocity constraint
-- Constraint{
--     context = ctx,
--     name    = "z_angular",
--     expr    = coord_z(getRotVec(rotation(feature[feature_i]['task_frame_i']))) - tgt_r_z*time,
--     K       = 0,
--     weight  = constant(1),
--     priority= 2
-- };

-- =========================== MONITORS ============================================
-- -- Force Monitors
-- monitor_F = loadstring("Monitor{ context=ctx, name='finish_force', upper=" .. max_F ..", actionname='exit', expr=sqrt(Fx*Fx + Fy*Fy + Fz*Fz)}")
-- monitor_T = loadstring("Monitor{ context=ctx, name='finish_torque', upper=" .. max_T ..", actionname='exit', expr=sqrt(Tx*Tx + Ty*Ty + Tz*Tz)}")
--
-- monitor_F()
-- monitor_T()
-- -- Distance Monitor
-- Monitor{
--         context=ctx,
--         name='finish_after_motion',
--         upper = 0.020,
--         actionname='exit',
--         expr = abs(traveled_distance_z)
-- };

-- Monitor{
--         context=ctx,
--         name='finish_after_stop',
--         lower = 0.0015,
--         actionname='exit',
--         expr = abs(velocity_y) + abs(velocity_z)
-- }
-- ============================== OUTPUT THROUGH PORTS===================================

roll_tf, pitch_tf, yaw_tf = getRPY(rotation(feature[feature_i]['tf']))

ctx:setOutputExpression("x_tf"		,coord_x(origin(feature[feature_i]['tf'])))
ctx:setOutputExpression("y_tf"		,coord_y(origin(feature[feature_i]['tf'])))
ctx:setOutputExpression("z_tf"		,coord_z(origin(feature[feature_i]['tf'])))
ctx:setOutputExpression("roll_tf"	,roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf"	,yaw_tf)

ctx:setOutputExpression("Fx"      ,Fx)
ctx:setOutputExpression("Fy"      ,Fy)
ctx:setOutputExpression("Fz"      ,Fz)
ctx:setOutputExpression("Tx"      ,Tx)
ctx:setOutputExpression("Ty"      ,Ty)
ctx:setOutputExpression("Tz"      ,Tz)

-- ctx:setOutputExpression("V_y"     ,velocity_y)
