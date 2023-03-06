-- ==============================================================================
-- Author: Rob Eyskens
-- email: <rob.eyskens@student.kuleuven.be>
-- Force simulation component
-- KU Leuven 2022
-- ==============================================================================

require("rttlib")
require("math")

tc=rtt.getTC()

rot_mat = rtt.provides("KDL"):provides("Rotation")

iface_spec = {
   ports={
      { name='force', datatype='array', type='out', desc="Simulated force" },
      { name='task_frame', datatype='array', type='in', desc="input task frame" },
      { name='force_sensor_frame', datatype='array', type='in', desc="input force sensor frame" },
   },
 
   properties={
      { name='initial_forces', datatype='array', desc="initial simulation force" },
      { name='object_char', datatype='array', desc="middle of object cylinder in xy plane, radius of cylinder, stiffness cylinder,tool_length" },
   }
}


iface=rttlib.create_if(iface_spec) 

time_elapsed = rtt.Variable("double")


fvals=rtt.Variable("array")
objvals=rtt.Variable("array")
forcevec=rtt.Variable("KDL.Vector")
middlevec=rtt.Variable("KDL.Vector")
robotpos=rtt.Variable("KDL.Vector")
difvec = rtt.Variable("KDL.Vector")
norm = rtt.Variable("double")
Force_dir = rtt.Variable("KDL.Vector")
del_p = rtt.Variable("double")
Force = rtt.Variable("KDL.Vector")
angles = rtt.Variable("KDL.Vector")
R = rtt.Variable("KDL.Rotation")
transformed_force = rtt.Variable("KDL.Vector")
result = rtt.Variable("double")

-- The Lua component starts its life in PreOperational, so
-- configureHook can be used to set stuff up.
function configureHook()
    iface=rttlib.create_if(iface_spec)
    local vals=iface.props.initial_forces:get():totab()
    fvals:fromtab( vals )
    iface.ports.force:write(fvals) 
    return true
end

function startHook()
    local vals=iface.props.initial_forces:get():totab()
    fvals:fromtab( vals )
    iface.ports.force:write(fvals) 
    time_elapsed = 0;
    
    return true
end


function updateHook()

    local objs=iface.props.object_char:get():totab()
    objvals:fromtab( objs )
    fs,pos=iface.ports.task_frame:read()
    fs1,force_sensor=iface.ports.force_sensor_frame:read()

    if fs~='NoData' then 
        local p = pos:totab()
        local dt=tc:getPeriod() 


        time_elapsed = time_elapsed + dt
        --print('Task frame;',pos)       
        
        middlevec.X = objvals[0]
        middlevec.Y = objvals[1]
        middlevec.Z = pos[2] 

        stiffness = objvals[3] --+ 2000*math.sin(time_elapsed)
        -- print(stiffness)
        
        --print("middlevec",middlevec)
        
        robotpos.X = pos[0]
        robotpos.Y = pos[1]
        robotpos.Z = pos[2]
        
        --print("Robotpos",robotpos)
        
        difvec = robotpos - middlevec
        norm = math.sqrt(difvec.X^2 + difvec.Y^2 + difvec.Z^2)
        
        --print("difvec",difvec)
        --print("norm",norm)
        
        Force_dir.X = difvec.X / norm
        Force_dir.Y = difvec.Y / norm
        Force_dir.Z = difvec.Z / norm
        
        --print("Force_dir",Force_dir)
        
        bearing_radius = 0.01;

        del_p = (objvals[2] + bearing_radius) - norm
        
        --print("del_p",del_p)
        
        result = difvec.X^2 + difvec.Y^2 + difvec.Z^2
        
        --print("result",result)
        --print("radius square",objvals[2]^2)

        if (result < ( (objvals[2] + bearing_radius)^2)) and (robotpos.Z<0.1) then 
            Force.X = stiffness*del_p*Force_dir.X
            Force.Y = stiffness*del_p*Force_dir.Y
            Force.Z = stiffness*del_p*Force_dir.Z
        else 
             Force.X = 0.0
             Force.Y = 0.0
             Force.Z = 0.0
        end
        
        angles.X = force_sensor[3]
        angles.Y = force_sensor[4]
        angles.Z = force_sensor[5]
        
        --print("angles", angles)
        --print("Force before transformation",Force)
        --print(R)
                
        R = rot_mat:RPY(angles.X,angles.Y,angles.Z)
        
        --print("Rotation",R) 
        
        force_transformed = rot_mat:Inverse(R) * (Force)
        
        --print("Force transformed",force_transformed)
        --print("Tracking angle",math.atan(force_transformed.Y, -force_transformed.X))
        
        --rtt.logl('Warning', "New iteration of force simulation finished.")
        
        fvals[0] = force_transformed.X -- + math.random()*0.1
        fvals[1] = force_transformed.Y -- + math.random()*0.1
        fvals[2] = force_transformed.Z -- + math.random()*0.1
        fvals[3] = 0.0  
        fvals[4] = 0.0  
        fvals[5] = 0.0    
            
    end
    iface.ports.force:write(fvals) 
    --print('Calculated;',fvals)
    --print("Written to output port",fvals)
end


function cleanupHook()
    rttlib.tc_cleanup()
end