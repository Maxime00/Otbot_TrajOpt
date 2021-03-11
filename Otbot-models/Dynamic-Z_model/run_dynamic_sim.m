%% Clear all variables, close all figures, and clear the command window
clearvars 
close all
clc

% Must launch this file from Dynamic folder to save files correctly
%
% Set up  : 
%       - Goal trajectory in 'goal_trajectory' folder
%       - disturbance and friction in 'solve_equations' folder
%       - traj_var 

% name of video and plot files
cd = '../Results/Inertia_1/Dynamic-Z_model/';
fn = 'zig-zag_ctc-control';

% set goal traj
traj_var = 'TrajOpt'; %load soln from TrajOpt solver
load ("F:\EPFL\IRI\matlab\Maxime\OptimTraj-master\Otbot\Results\Otbot\164-2_force-squared_simple_guess_to_9-9-0.00_trapezoid-48_nodes.mat")

% load dynamic model
load("m_struc")
load("sm_struc")

% Set integration parameters here
h = 0.001; % timestep in seconds
tf = 10 ;  % max time that we allow for reaching target
opts = odeset('MaxStep',1e-3); % Max integration time step

% Set desired start and end position 
% Set initial position and control
y0 = zeros(8,1);
y0(1:3)= [-9;-9;0];

%turn otbot
%  y0(3)= pi/180;

% Goal trajectory
% create_traj(m, soln, traj_var);

% initial position is deduced from goal trajecory
% p0 = goal_traj(0);
% phi0 = [0;0;-pi/2];
% pdot0 = goal_trajdot(0);
% phidot0 = sm.MIIKmatrix(p0(3),phi0(3)) * pdot0;
% 
% y0 = [p0 ; phi0 ;pdot0 ; phidot0];

%% solve and save trajectory

tic
sln = solve_eqns(sm, h, tf, opts, y0, soln, traj_var);
toc

%% save solution and plots 

plot_and_save(sln, cd, fn, m, soln, traj_var);

%% video simulation of solution

video_sim(sln, h, m, sm, cd, fn, soln, traj_var)

