%% Clear all variables, close all figures, and clear the command window
clearvars 
close all
clc

load("m_struc")
load("sm_struc")

% Set integration paramet ers here
h = 0.005; % timestep in seconds
tf = 10 ;  % max time that we allow for reaching target

% Set initial position
y0 = [0 ; 0 ; 0 ; 0 ; 0 ; 0 ] ;

% Final position
create_traj();

%% Solve and save trajectory
tic
sln = solve_eqns(sm, h, tf, y0);
toc

%% Save solution and plots 

plot_and_save(sln);

%% video simulation of solution

video_sim(sln, h, m)

