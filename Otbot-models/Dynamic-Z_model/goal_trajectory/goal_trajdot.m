function goal_traj_dot = goal_trajdot(t)
%GOAL_TRAJDOT
%    GOAL_TRAJ_DOT = GOAL_TRAJDOT(T)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    30-Dec-2020 17:12:36

t2 = pi./2.0;
t3 = t./2.0;
t4 = -t2;
t5 = t3+t4;
t6 = cos(t5);
t7 = sin(t5);
goal_traj_dot = [t6.*(-1.0./8.0)-t7.*(5.0./2.0);t6.*(5.0./2.0)-t7./8.0;1.0./2.0];
