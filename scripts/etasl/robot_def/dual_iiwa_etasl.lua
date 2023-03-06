require("context")
require("geometric")


local u=UrdfExpr();
u:readFromFile(rospack_find("planar_contour_following").."/robot_description/urdf/dual_iiwa/kia_setup.urdf")
u:addTransform("ft_sensor_frame_right","ft_sensor_frame_right","iiwa1_torso_link")
u:addTransform("ft_sensor_frame_left","ft_sensor_frame_left","world")
-- u:addTransform("tf_FT" ,"ft_sensor_frame_right","tcp_link")
u:addTransform("T_sensor_tcp" ,"tcp_link","ft_sensor_frame_right")
u:addTransform("T_world_sensor" ,"ft_sensor_frame_right", "iiwa1_torso_link")
u:addTransform("tcp_frame" ,"tcp_link","iiwa1_torso_link") -- name, child, parent
-- TODO Automate this part

-- u:addTransform("hook_oring_right","hook_oring_right","world")
-- u:addTransform("hook_oring_left","hook_oring_left","world")

-- if right_task_frame_name and left_task_frame_name then
--   print("looking for transformation")
--   print(right_task_frame_name)
--   print(left_task_frame_name)
--   u:addTransform("right_task_frame",right_task_frame_name,"world")
--   u:addTransform("left_task_frame",left_task_frame_name,"world")
--   print("transform ok")
-- elseif task_frame_name then
--   print("looking single arm")
--   u:addTransform("task_frame" ,task_frame_name, "world")
-- end

-- if idx then
--   u:addTransform("FT_sensor_frame" ,"ft_sensor_frame_"..idx, "world")
-- end

local r = u:getExpressions(ctx)
robot_joints=u:getAllJointNames()

-- FT_frame_right = r.ft_sensor_frame_right
-- FT_frame_left = r.ft_sensor_frame_left
FT_sensor_frame = r.ft_sensor_frame_right
-- T_tf_FT = r.tf_FT
T_sensor_tcp = r.T_sensor_tcp
T_world_sensor = r.T_world_sensor
tcp_frame = r.tcp_frame
-- TODO Automate this part

-- hook_oring_right = r.hook_oring_right
-- hook_oring_left = r.hook_oring_left

-- task_frame = r.task_frame

-- if right_task_frame_name and left_task_frame_name then
--   print("expression of transfromation")
--   right_task_frame = r.right_task_frame
--   left_task_frame = r.left_task_frame
--
-- elseif task_frame_name then
--   task_frame = r.task_frame
-- end
--
-- if idx then
--   FT_sensor_frame = r.FT_sensor_frame
-- end
