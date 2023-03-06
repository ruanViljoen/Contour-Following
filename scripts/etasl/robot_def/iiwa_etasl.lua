require("context")
require("geometric")

local u=UrdfExpr();
u:readFromFile(rospack_find("planar_contour_following").."/robot_description/urdf/iiwa/use_case_setup_iiwa.urdf")
u:addTransform("tool_frame","tool_frame","world")

local r = u:getExpressions(ctx)
robot_joints=u:getAllJointNames()

task_frame = r.tool_frame
FT_frame = r.tool_frame
