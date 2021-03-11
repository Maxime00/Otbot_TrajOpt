function u = control(t, y, sm, soln, traj_var)
% control function for dynamic model

% Set hyperparameters
[Kp, Kv, Ki] = control_hyper_parameters();

persistent i_old err_old t_old

% Get goal trajectory values from t
switch traj_var
    case 'fixed'
        p_traj = [-6;-6;-3*pi/4];
        p_traj_dot = [0;0;0];
        p_traj_dot_dot = [0;0;0];
    
    case 'circle'
        p_traj = goal_traj(t);
        p_traj_dot = goal_trajdot(t);
        p_traj_dot_dot = goal_trajdotdot(t);
        
    case 'square' 
        p_traj = square_traj(t);
        p_traj_dot = square_trajdot(t);
        p_traj_dot_dot = [0;0;0];
        
    case 'cst_acc' 
        a = 0.5;
        p_traj = [0.5*a*t^2;0;0];
        p_traj_dot = [a*t;0;0];
        p_traj_dot_dot = [a;0;0];    
   
    case 'M_traj' 
        [p_traj, p_traj_dot] = M_traj(t);
        p_traj_dot_dot = [0;0;0];    
    
    case 'Line_traj'
        [p_traj, p_traj_dot] = Line_traj(t);
        p_traj_dot_dot = [0;0;0];  
    
    case 'TrajOpt' 
        full_state = soln.interp.state(t);
        p_traj = full_state(1:3);
        p_traj_dot = full_state(4:6);
        full_state_dot = soln.problem.func.dynamics(t,full_state, soln.interp.control(t)); 
        p_traj_dot_dot = full_state_dot(4:6);
        
    otherwise
        p_traj = goal_traj(t);
        p_traj_dot = goal_trajdot(t);
        p_traj_dot_dot = goal_trajdotdot(t);
end

% convert y to x model form 
K_0 = 0;
x = sm.psi(K_0, y(3), y(6), y(7), y(8), y(1), y(4), y(2), y(5));

% Get useful variables from y
alpha = x(3);
varphi_p = x(6);

alpha_dot = x(9);
varphi_dot_p = x(12);

p = x(1:3);
pdot = x(7:9);

% Get matrices 
% MFIK = sm.MFIKmatrix(alpha,varphi_p);
Mbar_s = sm.Mbar(alpha,varphi_p);
Cbar_s = sm.Cbar(alpha,alpha_dot,varphi_p,varphi_dot_p);

% Get error
err_k = [p_traj(1:2) - p(1:2) ; angdiff(p(3),p_traj(3))];

% get integral term
if t > 0
    i_k = i_old + ((t - t_old)/2)*(err_k + err_old);
else 
    i_k = [0;0;0];
end

% update persistent variables
i_old = i_k;
t_old = t;
err_old = err_k;

% Get u prime
u_prime = Kp * err_k + Kv * ( p_traj_dot - pdot) + Ki * i_k +  p_traj_dot_dot;

% u = MFIK.' * ( Mbar_s *u_prime + Cbar_s*pdot) ; old model
u = Mbar_s *u_prime + Cbar_s*pdot ;

    
end