function dx = eqns4odeXmodel(t,x, soln, sm)
% equations dq = f(q,v)  -> q = (q;v) 

% Interpolate u from soln
u = interp1(soln(end).grid.time', soln(end).grid.control', t')';

xdot = x(7:12);

% Get qdotdot
xdotdot = sm.forward_dyn(x(3),x(9),u(2),u(3),u(1),x(6),x(11),x(12),x(10));

dx = [xdot ; xdotdot];

end