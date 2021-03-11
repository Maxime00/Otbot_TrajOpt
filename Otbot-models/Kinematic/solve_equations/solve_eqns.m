%% Solve equations of motion 
% Note: eqns.m defines the equations of motion to be solved by this script
% This function returns the time vector T and the solution Y

function sln = solve_eqns(sm, h, tf, y0)

tspan = 0 : h : tf; % from 0 to tmax with time step h

dydt = @(t,y) eqns(t, y, sm);

[t,y] = ode45(dydt, tspan, y0);

% Save K gain with trajectory
K = control_hyper_parameters();

sln.K = K;
sln.T = t;
sln.Y = y;

end


