function [p_traj, p_trajdot] = M_traj(t)
% SQUARE_TRAJ Summary of this function goes here
%   Detailed explanation goes here

% Polyline type 1 verical M letter
if t <= 5
    X =(1/5)*t;
    Y = 0;
    X_dot = (1/5);
    Y_dot = 0;
elseif t>5 && t<=10
    X = -(1/5)*(t-5) + 1;
    Y = (1/5)*(t-5);
    X_dot = -(1/5);
    Y_dot = (1/5);
elseif t>10 && t<=15
    X = (1/5)*(t-10);
    Y = (1/5)*(t-5);
    X_dot = (1/5);
    Y_dot = (1/5);
elseif t>15 && t<=20
    X = -(1/5)*(t-15) + 1;
    Y = 2;
    X_dot = -(1/5);
    Y_dot = 0;
else
    X = 0;
    Y = 2;
    X_dot = 0;
    Y_dot = 0;
end

p_traj = [X ; Y ; 0];
p_trajdot = [X_dot; Y_dot ;0];

end

