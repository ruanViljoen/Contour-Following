require("context")
require("geometric")

-- loading a model for the unversal robot UR10:
local u=UrdfExpr();
u:readFromFile(rospack_find("planar_contour_following").."/robot_description/urdf/franka_panda/use_case_setup_panda.urdf")
u:addTransform("ee","panda_link8","panda_link0")
u:addTransform("FT_frame","TCP_frame","panda_link0")
u:addTransform("TCP_frame","TCP_frame","panda_link0")
u:addTransform("T_TCP_FT","TCP_frame","TCP_frame")			-- Transformation from the hook to the load cell

local r = u:getExpressions(ctx)

robot_ee = r.ee
FT_frame = r.FT_frame
task_frame = r.TCP_frame
T_tf_FT = r.T_TCP_FT

robot_joints={"panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"}
