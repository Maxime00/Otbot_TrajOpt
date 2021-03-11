function create_traj(m)
% Creates a trajectory function and and associated derivatives used for control

syms t

% Choose here the desired trajectory function (maybe add switch)
% Go in function to modify parameters
% goal_traj = square_traj(t,m);
goal_traj = circle_traj(t,m);   

matlabFunction(goal_traj, 'File', 'goal_trajectory/goal_traj');

% get derivative of end traj 
goal_traj_dot = diff(goal_traj,t);
trajdot = matlabFunction(goal_traj_dot);
matlabFunction(goal_traj_dot, 'File', 'goal_trajectory/goal_trajdot');

% get second derivative of end traj 
goal_traj_dot_dot = diff(trajdot, t);
matlabFunction(goal_traj_dot_dot, 'File', 'goal_trajectory/goal_trajdotdot');

end
