function dz = eqns4ode(t,z, soln, sm)
% equations dq = f(q,v)  -> q = (q;v) 

% impose varphi_l(0) = 0
% will not work if phi_l is supposed to be at another position initially 
K_0 = 0; %-1 * sm.K(z(3,1), 0, z(7,1), z(8,1)); 

% get x from z
x = sm.psi(K_0, z(3), z(6), z(7), z(8), z(1), z(4), z(2), z(5));
xdot = x(7:12);

% Interpolate u from soln
u = interp1(soln(end).grid.time', soln(end).grid.control', t)';

% Get xdotdot
xdotdot = sm.forward_dyn(x(3),x(9),u(2),u(3),u(1),x(6),x(12),x(7),x(8));

dx = [xdot ; xdotdot];

% compute dz from dx 

dz = sm.L *dx;

end