require("context")
require("geometric")


local M = {}


local function trap_velprofile(maxvel,maxacc,s_start,s_end)
  sc = conditional( constant(0.5)-maxvel*maxvel/(constant(2)*maxacc) , maxvel*maxvel/(constant(2)*maxacc), constant(0.5) )
  s_p_3_4   = conditional(s_end-s , make_constant( sqrt( constant(2)*maxacc*(s_end-s) ) ) , constant(0))
  s_p_2_4   = conditional( (s_end-sc)-s , maxvel , s_p_3_4)
  s_p_1_4   = conditional((sc+s_start)-s , make_constant( sqrt( constant(2)*maxacc*(s-s_start) ) ) , s_p_2_4)
  s_p_0_4 = conditional(s_start-s , constant(0) , s_p_1_4)

  return s_p_0_4
end

local function trap_velprofile_tau(maxvel,maxacc,tau_start,tau_end)
  tauc = conditional( constant(0.5)*(tau_end-tau_start)-maxvel*maxvel/(constant(2)*maxacc) , maxvel*maxvel/(constant(2)*maxacc), constant(0.5)*(tau_end-tau_start) )
  tau_p_3_4   = conditional(tau_end-tau , make_constant( sqrt( constant(2)*maxacc*(tau_end-tau) ) ) , constant(0))
  tau_p_2_4   = conditional( (tau_end-tauc)-tau , maxvel , tau_p_3_4)
  tau_p_1_4   = conditional((tauc+tau_start)-tau , make_constant( sqrt( constant(2)*maxacc*(tau-tau_start) ) ) , tau_p_2_4)
  tau_p_0_4   = conditional(tau_start-tau , constant(0) , tau_p_1_4)

  return tau_p_0_4
end

-- auxiliary functions:
eps=constant(1E-14)
function close_to_zero( e, yes_expr, no_expr)
    return cached( conditional( e - eps, no_expr, conditional( -e+eps,  no_expr, yes_expr)) )
end
-- returns normalized vector and norm (taking into account that vector can be zero or very small)
function normalize( v )
    n  = cached( norm(v) )
    vn = cached( close_to_zero(n, vector(constant(1),constant(0),constant(0)), v/n) )
    return vn,n
end

function dead_zone(sign_0,dead_val)
   sign = conditional(abs(sign_0)-dead_val, sign_0 + conditional(sign_0, -dead_val, dead_val), constant(0))
   return sign
end



-- export functions
M.trap_velprofile = trap_velprofile
M.trap_velprofile_tau = trap_velprofile_tau
M.normalize = normalize
M.dead_zone = dead_zone
return M
