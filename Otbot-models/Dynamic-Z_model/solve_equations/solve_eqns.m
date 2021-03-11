%% Solve equations of motion 
% Note: eqns.m defines the equations of motion to be solved by this script
% This function returns the time vector T and the solution Y

function sln = solve_eqns(sm, h, tf, opts, y0, soln, traj_var)

tspan = 0 : h : tf; % from 0 to tmax with time step h

dydt = @(t,y) eqns(t, y, sm, soln, traj_var);

[t,y] = ode45(dydt, tspan, y0, opts);

% Save K gain with trajectory
[Kp, Kv, Ki ] = control_hyper_parameters();
K = [Kp ; Kv; Ki ];

sln.K = K;
sln.T = t.';
sln.Y = y.';
%add control to solution
u = zeros(3, length(t));

for i = 1:length(t) 
    u(:,i) = control(t(i), y(i, :).', sm, soln, traj_var);
end

sln.U = u;

end


