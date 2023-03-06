require("context")
require("geometric")

-- loading a model for the unversal robot UR10:
local u=UrdfExpr();
u:readFromFile(rospack_find("planar_contour_following").."/robot_description/urdf/ur10/use_case_setup_ur10.urdf")
-- u:addTransform("ee","tool0","base")
-- u:addTransform("FT_frame","load_cell_link","base")
u:addTransform("tcp_link","tcp_link","base")
u:addTransform("T_world_sensor" ,"load_cell_link", "base")
u:addTransform("T_sensor_tcp" ,"tcp_link","load_cell_link") -- name, child, parent
-- u:addTransform("T_TCP_FT","load_cell_link","TCP_frame")  -- Transformation from the hook to the load cell
u:addTransform("FT_sensor_frame","load_cell_link", "base")

local r = u:getExpressions(ctx)

-- robot_ee = r.ee
-- FT_frame = r.FT_frame
tcp_frame = r.tcp_link
-- T_tf_FT = r.T_TCP_FT
FT_sensor_frame = r.FT_sensor_frame
T_sensor_tcp = r.T_sensor_tcp
T_world_sensor = r.T_world_sensor

robot_joints={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}
