
function s( stringtable )
    local v = rtt.Variable("strings")
    v:fromtab(stringtable)
    return v
end

function d( floattable )
    local v = rtt.Variable("array")
    v:fromtab(floattable)
    return v
end

function define_property( comp, propname, proptype, propvalue, doc )
  local prop=rtt.Property(proptype,propname,doc)
  prop:set(propvalue)
  comp:addProperty(prop)
  return prop
end

function determine_robot(robot_name)
  local depl_robot_file
  local robot_etasl_dir
  if robot_name == "kinova_gen3" then
    depl_robot_file = "etasl_kinova_gen3"
    if simulation then
      robot_etasl_dir = robot_def_dir.."/kinova_gen3_simulation_etasl.lua"
    else
      robot_etasl_dir = robot_def_dir.."/kinova_gen3_etasl.lua"
    end

  elseif robot_name == "ur_10" then
    depl_robot_file = "etasl_UR10"
    robot_etasl_dir = robot_def_dir.."/ur10_etasl.lua"

  elseif robot_name == "kuka_iiwa" then
    depl_robot_file = "etasl_iiwa"
    robot_etasl_dir = robot_def_dir.."/iiwa_etasl.lua"

  elseif robot_name == "dual_kuka_iiwa" then
    depl_robot_file = "etasl_dual_iiwa"
    robot_etasl_dir = robot_def_dir.."/dual_iiwa_etasl.lua"

  elseif robot_name == "franka_panda" then
    depl_robot_file = "etasl_franka_panda"
    robot_etasl_dir = robot_def_dir.."/franka_panda_etasl.lua"

  elseif robot_name == "kuka_lwr" then
    depl_robot_file = "etasl_lwr"
    robot_etasl_dir = robot_def_dir.."/lwr_etasl.lua"

  else
    print("No valid robot_name was given")
  end
  return depl_robot_file,robot_etasl_dir
end
