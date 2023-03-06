require("context")
JSON = require ("JSON")
require("libexpressiongraph_spline")

if ilua~=nil then
    print( [[
    print(MotionModel) for help on using the motion model.
    function printMotion(ctx, model, pathvar_name, fv, startval, endval, step)
]])
end


--JSON.strictTypes = true  -- perfect round-trip is possible.
JSON.strictTypes = false  -- perfect round-trip is not possible.

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


--
-- prototype for a Degree of freedom class:
--    (style of obj definition: cf. https://www.lua.org/pil/16.1.html)
--

DegreeOfFreedom = {
    underlying_table={}
}

function DegreeOfFreedom:construct(table, pathvariable)
    local arg = {}
    setmetatable(arg, self)
    if table.name==nil then
        error("DegreeOfFreedom:underlying table has no 'name/=' field",2)
    end
    if table.s==nil then
        error("DegreeOfFreedom:underlying table has no 's' field ",2)
    end
    if table.modes==nil then
        error("DegreeOfFreedom:underlying table has no 'modes' field",2)
    end
    if table.mean==nil then
        error("DegreeOfFreedom:underlying table has no 'mean' field",2)
    end
    local L = #table.s
    if (L~=#table.mean) then
        error("DegreeOfFreedom:length of 's' is not equal to length of 'mean'",2)
    end
    for k,v in pairs(table.modes) do
        if L~=#v then
            error("DegreeOfFreedom:length of 's' is not equal to length of a mode (i.e. row of 'modes')",2)
        end
    end
    if extendedtype(pathvariable)~="expression_double" then
        print(pathvariable)
        error("DegreeOfFreedom: pathvariable should be a double expression")
    end
    arg._s      = table.s
    arg._splinetable = {}
    arg._pathvariable = pathvariable
    -- build spline table, first col is mean, other cols are modes
    for j = 1, #table.mean do
        arg._splinetable[j] = { table.mean[j] }
    end

    for i = 1, #table.modes do
        for j = 1, #table.modes[1] do
            arg._splinetable[j][i+1] = table.modes[i][j]
        end
    end
    arg._spline = createSpline(pathvariable, arg._s, arg._splinetable, 0)
    arg._mean = getSplineOutput(arg._spline,0)
    arg._output= {}
    for i=1,#table.modes,1 do
        arg._output[i] = getSplineOutput(arg._spline,i)
    end
    arg.underlying_table = table
    self.__index= self
    return arg
end

function DegreeOfFreedom:name()
    return self.underlying_table.name
end

function DegreeOfFreedom:nr_of_modes()
    return #self.underlying_table.modes
end

function DegreeOfFreedom:nr_of_samples()
    return #self.underlying_table.s
end

function DegreeOfFreedom:mode(i)
    if (i<1) or (i>self:nr_of_modes()) then
        error("DegreeOfFreedom:index out of bounds",2)
    end
    return self.underlying_table.modes[i]
end

function DegreeOfFreedom:expression(featurevar)
    if #featurevar~=#self._output then
        error("DegreeOfFreedom:expression requires a table array of the feature variables (starting index=1) ")
    end
    local e = self._mean
    for k,v in pairs(featurevar) do
        e = e + v*self._output[k]
    end
    return cached(e)
end

function DegreeOfFreedom:expression_value(featurevar, value)
    if #featurevar~=#self._output then
        error("DegreeOfFreedom:expression_value, argument 1 should be a table array of the feature variables (starting index=1) ")
    end
    if extendedtype(value)~="number" then
        error("DegreeOfFreedom:expression_value, argument 2 should be a number")
    end
    local e = constant(self._spline:getOutputValue(0,value))
    for k,v in pairs(featurevar) do
        e = e + v*constant(self._spline:getOutputValue(k,value))
    end
    return cached(e)
end




--
-- prototype for a Motionmodel class:
--    (style of obj definition: cf. https://www.lua.org/pil/16.1.html)
--

MotionModel = {
    underlying_table={},
    dof = {},
    hlp = [[
        MotionModel class:
            MotionModel:construct(tbl, pathvariable)
                call MotionModel:construct() for help
            MotionModel:nr_of_dof()
                returns the number of degrees of freedom
            MotionModel:nr_of_modes()
                returns the number of modes for each degree of freedom
            MotionModel:weights()
                returns the weights for each mode
            MotionModel:create_fv_and_constr(ctx,prefix,K_)
                call MotionModel:create_fv_and_constr for help
            MotionModel:dof(i)
                returns degree of freedom i, each degree of freedom is an object with
                the following methods:
                    DegreeOfFreedom:name()
                    DegreeOfFreedom:nr_of_modes()
                    DegreeOfFreedom:nr_of_samples()
                    DegreeOfFreedom:mode(i)
                        returns the table with the values to be interpolated for mode i
                    DegreeOfFreedom:expression(fv)
                        returns an expression in function of path variable and feature variables
                    DegreeOfFreedom:expression_value(fv, value)
                        returns an expression in function only feature variables, and with
                        the path variable a given value
            MotionModel:path_length()
                returns the path length of the mean curve.
    ]]
}

setmetatable(MotionModel, {
    __tostring = function (self)
        return MotionModel.hlp
    end
})

function MotionModel:construct(tbl, pathvariable)
   local hlp=[[
   usage:

   MotionModel:construct(tbl, pathvariable):
        constructs a MotionModel object from the data in the lua table tbl
        INPUT:
            tbl: lua table containing the specification
            pathvariable:  an (expressiongraph) expression corresponding to the path variable

        tbl is a lua table containing the specification, an example is given below:

                {
                    comment = "comments that have no other functions than be recorded in the json file",
                    description = "iteration of dof",
                    dof = {
                        {
                            mean = {
                                0, 0, 0
                            },
                            modes = {
                                { 0, 0.5, 1 },
                                { 1, 0.5, 0 }
                            },
                            name = "x",
                            s = {
                                0, 0.5, 1
                            }
                        },
                        {
                            mean = {
                                0, 0, 0
                            },
                            modes = {
                                { 0, 0, 0 },
                                { 0, 0, 0 },
                            },
                            name = "y",
                            s = { 0, 0.5, 1 }
                        }
                    },
                    mode_weights = { 1, 1 },
                    name = "motion1",
                    number_of_modes = 2,
                    number_of_samples = 3,
                    time_processed = "",
                    time_recorded = "",
                    path_length = 1.2
                }

      description, comment time_recorded and time_processed are optional, all  other variables are mandatory
   ]]
   local arg = {}
   if tbl==nil and pathvariable==nil then
       error(hlp,2)
   end
   if tbl.name==nil then
        error("MotionModel:underlying table has no 'name' field",2)
    end
    -- error checking ---
    -- description, comment, time_recorded, time_processed are optional fields
    tbl.description    = tbl.description or {}
    tbl.comment        = tbl.comment or {}
    tbl.time_recorded  = tbl.time_recorded or {}
    tbl.time_processed = tbl.time_processed or {}

    if tbl.mode_weights==nil then
        error("MotionModel:underlying table has no 'mode_weights' field",2)
    end
    if tbl.number_of_modes==nil then
        error("MotionModel:underlying table has no 'number_of_modes' field",2)
    end
    if #tbl.mode_weights~=tbl.number_of_modes then
        error("MotionModel:underlying table has number of 'mode_weights' not equal to 'number_of_modes'",2)
    end
    if tbl.number_of_samples==nil then
        error("MotionModel:underlying table has no 'number_of_samples' field",2)
    end
    if tbl.dof==nil or type(tbl.dof)~="table" then
        error("MotionModel:underlying table has no 'dof' table field",2)
    end
    arg._dof={}
    for k,v in pairs(tbl.dof) do
        print("dof "..v.name)
        local el = DegreeOfFreedom:construct(v,pathvariable)
        table.insert(arg._dof, el )
        if tbl.number_of_modes~=el:nr_of_modes() then
            error(string.format("MotionModel:number of modes (%d) for dof %d does not correspond to number_of_modes (%d)",el:nr_of_modes(), k, tbl.number_of_modes),2)
        end
        if tbl.number_of_samples~= el:nr_of_samples() then
            error(string.format("MotionModel:number of samples (%d) for dof %d does not correspond to number_of_samples (%d)",el:nr_of_samples(), k, tbl.number_of_samples),2)
        end
    end
    -- object & meta table ---
    arg.underlying_table = tbl
    setmetatable(arg, self)
    self.__index= self
    return arg
end

function MotionModel:nr_of_dof()
    return #self._dof
end

function MotionModel:dof(i)
    if (i<1) or (i>self:nr_of_dof()) then
        error("DegreeOfFreedom:index out of bounds",2)
    end
    return self._dof[i]
end

function MotionModel:nr_of_modes()
    return self.underlying_table.number_of_modes
end

function MotionModel:weights()
    return self.underlying_table.mode_weights;
end

function MotionModel:create_fv_and_constr( ctx, prefix, K_, scale_weights )
    local hlp=[[
    usage:

    function MotionModel:create_fv_and_constr( ctx, prefix, K_ )
        ctx:  Context in which constraints will be created
        prefix: prefix for feature variables (prefix..'var'..i) and constraints (prefix ..'constr'..i)
        K_    : control constant for the constraints
    ]]
    if (scale_weights==nil) then
        scale_weights = 1
    end
    if (ctx==nil) and (prefix==nil) and (K_==nil) then
        error(hlp,2)
    end
    if extendedtype(ctx)~="Context" then
        error("first argument should be a Context, "..hlp,2)
    end
    if extendedtype(prefix)~="string" then
        error("second argument should be a prefix string, "..hlp, 2);
    end
    if extendedtype(K_)~="number" then
        error("third argument should be a control constant (number), "..hlp, 2);
    end
    local fv       = {}
    local fv_names = {}
    local w = self:weights()
    for i = 1, self:nr_of_modes(),1 do
        fv_names[i] = prefix.."var"..i
        -- if i == 3 then
            -- fv[i] = Variable{context=ctx, name=fv_names[i], type="feature", initial=5};
        -- else
            fv[i] = Variable{context=ctx, name=fv_names[i], type="feature"};
        -- end
        Constraint{ context=ctx, name=prefix.."constr"..i, expr=fv[i], target=0.0, weight = w[i]*scale_weights, K = K_ };
    end
    -- Constraint{ context=ctx, name="constr_fv_test_1", expr=fv[1], target=0.109668, weight = 50, K = 4};
    -- Constraint{ context=ctx, name="constr_fv_test_2", expr=fv[2], target=0.0860862, weight = 50, K = 4};
    -- Constraint{ context=ctx, name="constr_fv_test_3", expr=fv[3], target=0.612198, weight = 50, K = 4};
    -- Constraint{ context=ctx, name="constr_fv_test_4", expr=fv[4], target=0.0714154, weight = 50, K = 4};

    return fv, fv_names;
end


-- !! NIL if no path length available !!!
function MotionModel:path_length()
    return self.underlying_table.path_length
end

function printMotion(ctx, model, pathvar_name, fv, startval, endval, step)
    local ndx = ctx:getScalarNdx(pathvar_name)
    local e = {}
    for i = 1, model:nr_of_dof() do
        e[i] = model:dof(i):expression(fv)
    end

    local s
    for s=startval,endval,step do
        line = s
        for i = 1, model:nr_of_dof() do
            e[i]:setInputValue(ndx, s)
            line = line .. "\t" .. e[i]:value()
        end
        print(line)
    end
end

function printDistribution(filename, ctx, model, pathvar_name, fv, startval, endval, step,scale)
    if scale==nil then
        scale=1
    end
    local file = io.open(filename,"w+")
    local ndx = ctx:getScalarNdx(pathvar_name)
    local e = {}
    for i = 1, model:nr_of_dof() do
        e[i] = model:dof(i):expression(fv)
    end

    local s
    for s=startval,endval,step do
        file:write(s)
        for i = 1, model:nr_of_dof() do
                    e[i]:setInputValue(ndx, s)
                    file:write("\t")
                    file:write(e[i]:value()*scale)
         end
         file:write("\n")
    end
    file:close();
end
