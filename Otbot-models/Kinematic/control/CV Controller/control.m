function u = control(t, y)
% Simple control function 
% PD for kinematic model

K = control_hyper_parameters();

p_traj = goal_traj(t);
p_traj_dot = goal_trajdot(t);

u = p_traj_dot + K * (p_traj - y(1:3));

end