#!/bin/bash

echo "collison_info={" > collision_info.lua 
for i in link_?.stl;
do
    echo "processing $i"
    meshlabserver -i $i -o "${i%.stl}.obj"
    rosrun expressiongraph_collision extract_capsule "${i%.stl}.obj" "tmp.obj" >> collision_info.lua
    meshlabserver -i tmp.obj -s convex_hull.mlx -o "${i%.stl}_capsule.obj" 
done
echo "}" >> collision_info.lua
cat collision_info.lua

