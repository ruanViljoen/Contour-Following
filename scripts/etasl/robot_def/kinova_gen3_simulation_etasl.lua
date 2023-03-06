require("context")
require("geometric")

local u=UrdfExpr();
u:readFromFile(rospack_find("planar_contour_following").."/robot_description/urdf/kinova/use_case_setup_gen3.urdf")
u:addTransform("tool_frame","tool_frame","base_link")

u:addTransform("right_inner_finger_pad","right_inner_finger_pad","base_link") -- This must be added or eTaSL will exclude the gripper from its calculations (to optimize)
u:addTransform("left_inner_finger_pad","left_inner_finger_pad","base_link")
u:addTransform("end_effector_link","end_effector_link","base_link")
robot_joints={"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7","finger_joint"};

local r = u:getExpressions(ctx)
task_frame = r.tool_frame
FT_frame = r.tool_frame
-- right_inner_finger_pad = r.right_inner_finger_pad
-- left_inner_finger_pad = r.left_inner_finger_pad
