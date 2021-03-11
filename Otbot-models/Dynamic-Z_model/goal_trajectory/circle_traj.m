function p_end = circle_traj(t,m)
% Parameters for end goal trajectories 

% a,b are coordinates for the center of the circle
% r is the radius of the trajectory
% T is the period (time the Otobot takes to do a full circle)
% w is the angular speed of the Otbot ( w = 2*pi/T)

% r_m is the radius of the m point trajectory
%l1 is the distance between M and P 

a = 4;
b = 6;
% r = 3;
w = 0.5;
ang_offset = -pi/2;

l1 = m.l_1;
r_m = 5;

% coordinates for circle traj
x = a + r_m * cos(w * t + ang_offset) - l1 * sin(w * t + ang_offset);
y = b + r_m * sin(w * t + ang_offset) + l1 * cos(w * t + ang_offset);
alpha = w*t; %0*t   %w*t
p_end = [x ; y ; alpha];
end

