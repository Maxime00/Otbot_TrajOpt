function p_end = circle_traj(t)
% Parameters for end goal trajectories 

% a,b are coordinates for the center of the circle
% r is the radius
% T is the period (time the Otobot take sto do a full circle)

a = 5;
b = 5;
r = 2;
T = 8;

% coordinates for circle traj
x = a + r * sin(2*pi/T * t);
y = b + r * cos(2*pi/T * t);
alpha = 0*t; %2*pi/T * t + pi/2;

p_end = [x ; y ; alpha];
end

