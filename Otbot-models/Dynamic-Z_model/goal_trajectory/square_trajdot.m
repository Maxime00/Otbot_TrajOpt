function p_end = square_trajdot(t)
% SQUARE_TRAJ Summary of this function goes here
%   Detailed explanation goes here

w = 0.8; % speed 
l = 5; % length

if t <= l
    x = w;
    y = 0;
    alpha = 0;
    
elseif t>l && t<=2*l
    x = 0;
    y = w;
    alpha = 0;
    
elseif t>2*l && t<=3*l
    x = -w;
    y = 0;
    alpha = 0;
    
elseif t>3*l && t<=4*l
    x = 0;
    y = -w;
    alpha = 0;
    
end

p_end = [x ; y ; alpha];


end

