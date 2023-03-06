-- ==============================================================================
-- Author: Federico Ulloa
-- email: <federico.ulloarios@student.kuleuven.be>
-- Helper to process the Json information from CAD
-- KU Leuven 2021
-- ==============================================================================

JSON = require("JSON")

-- ===================== VARIABLE DEFINITION =======================
-- TODO find a better (Good) way to have status variables
-- Possible world_position_status {"perception_error", "one_DOF_error", "best_approx"}
world_position_status = "perception_error"

function read_to_string(file)
    local f = io.open(file, "rb")
    local content = f:read("*all")
    f:close()
    return content
end

function read_json( str )
    obj,pos,err = JSON:decode(str)
    if err then
        error(err);
    end
    return obj
end

etasl_application_dir = rtt.provides("ros"):find("planar_contour_following")

local part_locations_str = read_to_string(etasl_application_dir.."/data/node_properties.json")
part_locations_tb = read_json(part_locations_str)

local user_defined_relations_str = read_to_string(etasl_application_dir.."/data/ordering_relations.json")
user_defined_relations_tb = read_json(user_defined_relations_str)

local insertion_relations_str = read_to_string(etasl_application_dir.."/data/insertion_relations.json")
insertion_relations_tb = read_json(insertion_relations_str)


function get_node_keys()
  local node_keys = {}
  local n = 1
  for k, tbl in pairs(part_locations_tb) do
    if tbl.part_name ~= "" then
      node_keys[n] = k
      n=n+1
    end
  end
  return node_keys
end

assembly_node_keys = get_node_keys()

function vector_scalar_multiplication(vector, magnitud)
  local m = rtt.Variable("double")
  m:assign(magnitud)
  return m*vector
end

function get_part_KDLFrame(part)
  local f = rtt.Variable("KDL.Frame")
  local pos_table = part_locations_tb[part].frame_position
  f:fromtab{p=pos_table.p,M=pos_table.M}
  return f
end

function get_relation_key(part1, part2)
  local obj1_flag = false
  local obj2_flag = false
  for key, value in pairs(insertion_relations_tb.Relations) do
    for k, obj in pairs(value.Objects) do
      if k == part1 then
        obj1_flag = true
      elseif k == part2 then
        obj2_flag = true
      end
    end
    if obj1_flag and obj2_flag then
      return key
    end
  end
  return nil
end

function get_parts_insertion_vector(part1, part2)
  local relation_index = get_relation_key(part1, part2)
  if relation_index then
    local insertion_vector = rtt.Variable("KDL.Vector")
    local insertion_length = insertion_relations_tb.Relations[relation_index].InsertionLength
    local insertion_axis = insertion_relations_tb.Relations[relation_index].InsertionDirection
    insertion_vector:fromtab(insertion_axis)
    insertion_vector = vector_scalar_multiplication(insertion_vector, insertion_length)
    return insertion_vector
  end
  return nil
end

function get_parts_insertion_vector_by_index(relation_index)
  local insertion_vector = rtt.Variable("KDL.Vector")
  local insertion_length = insertion_relations_tb.Relations[relation_index].InsertionLength
  local insertion_axis = insertion_relations_tb.Relations[relation_index].InsertionDirection
  insertion_vector:fromtab(insertion_axis)
  insertion_vector = vector_scalar_multiplication(insertion_vector, insertion_length)
  return insertion_length, insertion_vector
end

function get_parts_insertion_frame_by_index(relation_index)
  local insertion_frame = rtt.Variable("KDL.Frame")
  local insertion_point = insertion_relations_tb.Relations[relation_index].AxisPoint
  -- local insertion_axis = insertion_relations_tb.Relations[relation_index].InsertionDirection
  insertion_frame:fromtab{p=insertion_point}
  return insertion_frame
end

function get_initial_part()
  for k, _ in pairs(user_defined_relations_tb) do
    return k
  end
end

function is_child(parent_tag, child_tag)
  if not user_defined_relations_tb[parent_tag] then return false end
  for _, node_tag in pairs(user_defined_relations_tb[parent_tag]) do
    if node_tag == child_tag then
      return true
    end
  end
  return false
end

function get_parent_nodes(child_node_key)
  local parent_nodes = {}
  for _, node_key in pairs(assembly_node_keys) do
    if is_child(node_key, child_node_key) then
      parent_nodes[node_key] = true
    end
  end
  return parent_nodes
end

function get_possible_assembly_parts(assembled_parts)
  local possible_parts = {}
  for _, node_key in pairs(assembly_node_keys) do
    local all_parents_assembled = true
    if not assembled_parts[node_key] then
      parent_nodes = get_parent_nodes(node_key)
      for parent_node, _ in pairs(parent_nodes) do
        if not assembled_parts[parent_node] then all_parents_assembled = false end
      end
      if all_parents_assembled then possible_parts[node_key] = true end
    end
  end
  return possible_parts
end

function get_relations(node_tag, assembled_parts)
  local relations = {}
  -- Iterate over all relations
  for key, value in pairs(insertion_relations_tb.Relations) do
    local relation_flag = false
    local assembled_flag = false
    -- Iterate over relations in key
    for k, obj in pairs(value.Objects) do
      -- Check if node tag is in the relation
      if k == node_tag then
        relation_flag = true
      -- Check if the other node is already assembled
      elseif assembled_parts[k] == true then
        assembled_flag = true
      end
    end
    if relation_flag and assembled_flag then
      table.insert(relations, key)
    end
  end
  return relations
end


function update_part_world_position(node_key, assembled_pose)

end









-- final
