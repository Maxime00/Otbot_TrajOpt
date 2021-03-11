function [p_traj, p_trajdot] = Line_traj(t)
% SQUARE_TRAJ Summary of this function goes here
%   Detailed explanation goes here

w = 3/5; % speed
a = 5; % time_segment

% Polyline type line back and forth
if t <= a
    X =w*t;
    Y = 0;
    X_dot = w;
    Y_dot = 0;
elseif t>a && t<=2*a
    X = -w*(t-a) + w*a;
    Y = 0;
    X_dot = -w;
    Y_dot = 0;
elseif t>2*a && t<=3*a
    X = w*(t- 2*a);
    Y = 0;
    X_dot = w;
    Y_dot = 0;
elseif t>3*a && t<=4*a
    X = -w*(t- 3*a) + w*a;
    Y = 0;
    X_dot = -w;
    Y_dot = 0;
else
    X = 0;
    Y = 0;
    X_dot = 0;
    Y_dot = 0;
end

p_traj = [X ; Y ; 0];
p_trajdot = [X_dot; Y_dot ;0];

end

