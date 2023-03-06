require("context")
require("geometric")

local u=UrdfExpr();
u:readFromFile(rospack_find("planar_contour_following").."/robot_description/urdf/kinova/use_case_setup_gen3.urdf")
u:addTransform("tool_frame","tool_frame","base_link")

robot_joints={"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"};

local r = u:getExpressions(ctx)
task_frame = r.tool_frame
FT_frame = r.tool_frame
