function dy = eqns(t, y, sm, soln, traj_var)
% equations dq = f(q,v)  -> y = (q;v) 

% Get control variables for this step
u = control(t, y, sm, soln, traj_var); 
%u = control_check(t, ctrl);
%u = goal_trajdotdot(t);
%u = [1;-1;0];
%u = soln.interp.control(t);

% z model 
persistent K_0

if t == 0
    K_0 = -1 * sm.K(y(3), 0, y(7), y(8));
end

% get x from y
x = sm.psi(K_0, y(3), y(6), y(7), y(8), y(1), y(4), y(2), y(5));

xdot = x(7:12);

% Get xdotdot
xdotdot = sm.forward_dyn(x(3),x(9),u(2),u(3),u(1),x(6),x(12),x(7),x(8));

dx = [xdot ; xdotdot];

% compute dz from dx 
dy = sm.L *dx;


%dy = [qdot ; qdotdot];

end