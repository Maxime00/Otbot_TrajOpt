function [CaP,CbP, CaPGrad, CbPGrad] = pathCst(t,z,u,sm, XYbound, motor_torques)
%PATHCST implements path constraints with numCol = size(t) 
% one row per obstacle
% CaP is inequality constraint, CbP is equality constraint 

% get_round_obstacles % call array with all obstacles positions, radius

% grab obstacles from function wher tehy're defined
rObst = get_round_obstacles();
rObst_t = get_round_obstacles_over_time(t);

% Size of Otbot (size of protection radius around otbot)
r_otbot = 0.5;

% initialize variables
if strcmp(motor_torques, 'constant') 
   CaP = zeros(2+size(rObst,1)+size(rObst_t,1),size(t,2)); 
    CbP = zeros(2+size(rObst,1)+size(rObst,1),size(t,2));
elseif strcmp(motor_torques, 'vel-dep') 
    CaP = zeros(2+2*size(u,1)+size(rObst,1)+size(rObst_t,1),size(t,2));  
    CbP = zeros(2+2*size(u,1)+size(rObst,1)+size(rObst,1),size(t,2)); 
end

%% loop for limits in X Y direction
for i = 1:size(CaP,2) % for each node -> column of CaP    
    CaP(1:2,i) = abs(z(1:2,i))+r_otbot - XYbound(1:2);   
end

%% loop for velocity dependent torque bounds
%%% to use, uncomment this part and add +2*size(u,1) to CaP, CbP, CapGrad,
%%% CbPGrad row size
% fixed parameters
% t_stall = 2; %Nm, stall torque
% no_load_speed = 50000*(2*pi/60); % rad/s, 0 load speed
% s = - (t_stall/no_load_speed); % slope of velocity-dep torque bounds
% N_wheel = 50; %gear box factor wheel motor
% N_pivot = 150; %gear box factor pivot motor
% 
% for i = 1:size(CaP,2) % for each node -> column of CaP
%     phi_dot = sm.MIIKmatrix(z(3,i), z(7,i)) * z(4:6,i);
%     t_max_r = N_wheel* (t_stall + s*N_wheel*phi_dot(1));
%     t_max_l = N_wheel* (t_stall + s*N_wheel*phi_dot(2));
%     t_max_p = N_pivot* (t_stall + s*N_pivot*phi_dot(3));
%     t_max = [t_max_r; t_max_l; t_max_p];
%     t_min_r = N_wheel* (-t_stall + s*N_wheel*phi_dot(1));
%     t_min_l = N_wheel* (-t_stall + s*N_wheel*phi_dot(2));
%     t_min_p = N_pivot* (-t_stall + s*N_pivot*phi_dot(3));
%     t_min = [t_min_r; t_min_l; t_min_p];
%     CaP(3:5,i) = u(:,i) - t_max; 
%     CaP(6:8,i) = t_min - u(:,i); 
% end


%%% loop for round obstacles
for k = 1:size(rObst,1) % for each obstacles -> row of CaP
    for i = 1:size(CaP,2) % for each node -> column of CaP
        CaP(k+2,i) = (r_otbot + rObst(k,3))^2 - ( (rObst(k,1) -z(1,i))^2 + (rObst(k,2) -z(2,i))^2); %+2*size(u,1)
    end
end

%%% loop for round obstacles over time
for k = 1:size(rObst_t,1) % for each obstacles -> row of CaP
    for i = 1:size(CaP,2) % for each node -> column of CaP
        CaP(k+2+size(rObst,1),i) = (r_otbot + rObst_t(k,i,3))^2 - ( (rObst_t(k,i,1) -z(1,i))^2 + (rObst_t(k,i,2) -z(2,i))^2);%+2*size(u,1)
    end
end


if nargout == 4 % Analytic gradients
    
    if strcmp(motor_torques, 'constant') 
        CaPGrad = zeros(2+size(rObst,1)+size(rObst_t,1), 1 + size(z,1) + size(u,1), size(t,2));
        CbPGrad = zeros(2+size(rObst,1)+size(rObst_t,1), 1 + size(z,1) + size(u,1), size(t,2));
    elseif strcmp(motor_torques, 'vel-dep') 
        CaPGrad = zeros(2+size(u,1)*2+size(rObst,1)+size(rObst_t,1), 1 + size(z,1) + size(u,1), size(t,2));
        CbPGrad = zeros(2+size(u,1)*2+size(rObst,1)+size(rObst_t,1), 1 + size(z,1) + size(u,1), size(t,2));
    end

    % for limits in X Y direction
    CaPGrad(1, 2, :) = 1;
    CaPGrad(2, 3, :) = 1;
end

end

