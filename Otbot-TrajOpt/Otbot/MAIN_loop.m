% MAIN.m
%
% Solve the Otbot problem  --  any parametrization
%

clc; clear;
close all
addpath ../../

load("m_struc") % structure with mass parameters
load("sm_struc") % structure with dynamics equations

%%% strategy for initial guess
initial_guess = 'no_guess_'; %'simple_guess_';%'no_guess_'

%%% trajectory optimization method
% method = 'test' ; % <-- testing stuff out
% method = 'trapezoid'; %  <-- this is robust, but less accurate
% method = 'trapGrad'; %  <-- analytical gradients enabled
% method = 'hermiteSimpson'; %  <-- this is robust, but less accurate
% method = 'hermiteSimpGrad'; %  <-- analytical gradients enabled
% method = 'direct'; %  <-- this is robust, but some numerical artifacts
% method = 'rungeKutta';  % <-- slow, gets a reasonable, but sub-optimal soln
% method = 'orthogonal';    %  <-- this usually finds bad local minimum
%method = 'gpops';      %  <-- fast, but numerical problem is maxTorque is large

XYbound = [11;11]; % max bound for X and Y position 
start_pos = [0;0;0]; % starting position of the robot

%%% general parameters to display in file name
methods = ["trapezoid",];%, ]; "trapGrad","trapezoid" ,"hermiteSimpson"
final_states = [[10;10;0]];%[-8;-9;0]];,[-6;4;0][-6;-6;-3*pi/4],[-6;6;3*pi/4],[6;-6;-pi/4],[6;6;pi/4]
nodes  = [16];%,,40,32,24,16,8,30,40,50,60];   
obj_funcs = ["time_"];%,]; % ,"force-squared_","force-derivative_","time-forceterm_",,"time_","time_","time-forceterm-phi-p_""force-derivative_"
loop_name = '101_';

% Motor bounds 
motor_torques = 'constant'  ; %'constant', 'vel-dep'

if strcmp(motor_torques, 'constant') 
    maxForceWheel = 75;% %Nm %Maximum actuator forcesinf;%
    maxForcePivot = 230;% %Nm  %Maximum actuator forcesinf;%
end

% Choose what to plot 
make_video = 'no';
show_torques = 'yes';
show_dyn_err = 'yes';
show_kin_err = 'no';
show_traj_err = 'no';
show_traj_comp = 'no';
show_ctrl_eff = 'no';
show_path_cstr = 'no';
make_comp_time_plots = 'no'; % comp. time for each method for varying n_nodes
make_dyn_err_plots = 'no';


% array for storing multiple metric values for varying number of nodes
time_measures = zeros(length(methods),length(nodes));
max_dyn_err = zeros(8,length(nodes),length(methods));
avg_dyn_err = zeros(8,length(nodes),length(methods));
std_dyn_err = zeros(8,length(nodes),length(methods));

fig_number = 0;  % value for plots 

%%
for method = methods
    for final_state = final_states
        for obj_func = obj_funcs
            fig_number = fig_number+1; % to plot all lines in same figure
            for n_nodes = nodes
                %for maxSpeed = max_speed
                    % convert to string for file name
                    f_state_str = sprintf('%.0f-%.0f-%.2f', final_state(1), final_state(2), final_state(3));
                    nodes_str = sprintf('%.0f',n_nodes);
                    %f_str = sprintf('%.2f',maxSpeed);

                    % file name 
                    file_to_save = strcat('Otbot/Results/Otbot/',loop_name,obj_func,initial_guess,'to_',f_state_str,'_',method,'-',nodes_str,'_nodes.mat');
                    [folder, basefn, extension] = fileparts(file_to_save);
                    plotfn = strrep(basefn, '_', ' ');

                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
                    %                     Set up function handles                             %
                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

                    problem.func.dynamics = @(t,x,u)( OtbotDynamics(x,u,sm) );
                    problem.func.pathObj = @(t,x,u)( OtbotPathObj(t,u,obj_func) );

                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
                    %                     Set up problem bounds                               %
                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

                    problem.bounds.initialTime.low = 0;
                    problem.bounds.initialTime.upp = 0;
                    problem.bounds.finalTime.low = 0.001;
                    problem.bounds.finalTime.upp = 10;


                    problem.bounds.initialState.low = [start_pos;0;0;0;0;0];
                    problem.bounds.initialState.upp = [start_pos;0;0;0;0;0];
                    problem.bounds.finalState.low = [final_state;0;0;0;-inf;-inf];
                    problem.bounds.finalState.upp = [final_state;0;0;0;inf;inf];

                    problem.bounds.state.low = [-XYbound(1);-XYbound(2);-inf;-inf;-inf;-inf;-inf;-inf];
                    problem.bounds.state.upp = [XYbound(1);XYbound(2);inf;inf;inf;inf;inf;inf];

                    problem.bounds.control.low = [-maxForceWheel;-maxForceWheel;-maxForcePivot];
                    problem.bounds.control.upp = [maxForceWheel;maxForceWheel;maxForcePivot];

                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
                    %                     Set up obstacles                                    %
                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

                    problem.func.pathCst = @(t,x,u)( pathCst(t,x,u,sm, XYbound, motor_torques)); 
                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
                    %                    Initial guess at trajectory                          %
                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

                    switch initial_guess
                        case 'no_guess_'
                            problem.guess.time = [0,5];
                            problem.guess.state = [problem.bounds.initialState.low, [final_state;0;0;0;0;0]];%;0;0;0;0]];
                            problem.guess.control = [[0;0;0], [0;0;0]];

                        case 'simple_guess_'
                            problem.guess.time = [0,3,6,10];
                            problem.guess.state = [problem.bounds.initialState.low, [0;-8;0;0;0;0;0;0] ,  [0;8;0;0;0;0;0;0] ,[final_state;0;0;0;0;0]];%;0;0;0;0]];
                            problem.guess.control = [[0;0;0],[0;0;0],[0;0;0],[0;0;0] ];

                        case 'CTC_' 
                            %%% use CTC traj as first guess
                            problem.guess.time = sln.T;
                            problem.guess.state = sm.L *sln.Y;
                            problem.guess.control = sln.U;

                        case 'prev_traj_'
                            % %%%% use previous optimal rtajectory as start 
                            load(file_to_load);
                            problem.guess.time = linspace(soln(end).grid.time(1), soln(end).grid.time(end), 150);
                            problem.guess.state = soln(end).interp.state(problem.guess.time);
                            problem.guess.control = soln(end).interp.control(problem.guess.time);

                    end

                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
                    %                         Solver options                                  %
                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
                    %

                    %%%% Method-independent options: 
                    problem.options(1).nlpOpt = optimset(...
                     'Display','iter',...   % {'iter','final','off'}
                     'TolFun',1e-4,...
                     'MaxFunEvals',1e6,...
                     'MaxIter', 1e3); %,...'TolX', 1e-15


                    switch method
                        case 'trapezoid'
                            problem.options(1).method = 'trapezoid';
                            problem.options(1).trapezoid.nGrid = n_nodes;

    %                         problem.options(2).method = 'trapezoid';
    %                         problem.options(2).trapezoid.nGrid = 2*n_nodes;

                        case 'trapGrad'
                            problem.options(1).method = 'trapezoid'; % Select the transcription method
                            problem.options(1).trapezoid.nGrid = n_nodes;  %method-specific options
                            problem.options(1).nlpOpt.GradConstr = 'on';
                            problem.options(1).nlpOpt.GradObj = 'on';
                            problem.options(1).nlpOpt.DerivativeCheck = 'on';

                        case 'hermiteSimpson'
                            problem.options.method = 'hermiteSimpson';
                            problem.options.hermiteSimpson.nSegment = n_nodes;

                        case 'hermiteGrad'
                            problem.options.method = 'hermiteSimpson';
                            problem.options.hermiteSimpson.nSegment = n_nodes;
                            problem.options.nlpOpt.GradConstr = 'on';
                            problem.options.nlpOpt.GradObj = 'on';
                            problem.options.nlpOpt.DerivativeCheck = 'on';

                        case 'orthogonal'
                            problem.options.method = 'chebyshev';
                            problem.options.chebyshev.nColPts = n_nodes;

                       case 'rungeKutta'
                            problem.options(1).method = 'rungeKutta';
                            problem.options(1).defaultAccuracy = n_nodes;

                        case 'gpops'
                            problem.options(1).method = 'gpops';

                    end

                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
                    %                            Solve!                                       %
                    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

                    soln = optimTraj(problem);

                    % Save and print computation time 
                    time_measures(find(methods == method),find(nodes ==n_nodes))= 0;
                    for i = 1:size(soln)
                        time_measures(find(methods == method),find(nodes ==n_nodes))= time_measures(find(methods == method),find(nodes ==n_nodes))+ soln(i).info.nlpTime;
                    end
                    fprintf('OptTraj took %.3f seconds to solve problem \t', time_measures(find(methods == method),find(nodes ==n_nodes)));
                    %%%  Save solution
                    save(file_to_save, "soln", "soln");

                    %%%% Plots

                    t = linspace(soln(end).grid.time(1), soln(end).grid.time(end), 1500);
                    z = soln(end).interp.state(t);
                    u = soln(end).interp.control(t);

                    %% Kinematic error 

                    if strcmp(show_kin_err,'yes')
                        cstr_1 = zeros(3,length(t));

                        % convert z back to x format for kin constraints
                        K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1));
                        x = sm.psi(K_0, z(3,:), z(6,:), z(7,:), z(8,:), z(1,:), z(4,:), z(2,:), z(5,:));

                        for i = 1:length(t)
                            cstr_1(:,i) = sm.newJmatrix(x(3,i), x(6,i)) * x(7:12,i);
                        end
                        %  cstr_2 = r/(2*l_2) *(phi_r(t) -phi_r(0)) -r/(2*l_2) *(phi_l(t) -phi_l(0)) + (phi_p(t) - phi_p(0)) -(alpha(t) - alpha(0) )
                        %cstr_2 = m.r/(2*m.l_2) *(x(4,:) - x(4,1) + x(5,1) - x(5,:)) +(x(6,:) - x(6,1)) -(x(3,:) - x(3,1));
                        cstr_2 = sm.K(z(3,:), x(5,:), z(7,:), z(8,:)) + K_0;

                        % Kinematic error Plot
                        figure(3000); 
                        plot(t,cstr_1(1,:), t, cstr_1(2,:), t, cstr_1(3,:), t, cstr_2);  
                        legend('$J_1$', '$J_2$','$J_3$','$g(\dot{q})$', 'Interpreter', 'latex');
                        title(strcat('Kinematic error ', plotfn) )
                        xlabel('Time [s]')
                        ylabel('Kinematic Errors')
                        saveas(gcf, strcat('Otbot/Results/Otbot/plots/',loop_name,'kin_error_',obj_func, 'to_', f_state_str,'_',nodes_str,'_nodes.png') )
                        clear gcf;
                    end

                    %% Torque Plots
                    if strcmp(show_torques, 'yes')
                        figure(3*fig_number); 
                        hold on; grid on;
                        plot(t,u(1,:), 'DisplayName', strcat(nodes_str, ' nodes') );
                        legend
                        title(strcat('Torque right wheel',{' '}, obj_func, ' to',{' '}, f_state_str) )
                        xlabel('Time [s]')
                        ylabel('$\tau_r$ [Nm]', 'Interpreter', 'latex')

                        figure(3*fig_number-1); 
                        hold on; grid on;
                        plot(t,u(2,:), 'DisplayName',strcat(nodes_str, ' nodes') ); 
                        legend
                        title(strcat('Torque left wheel',{' '},obj_func, ' to',{' '}, f_state_str) )
                        xlabel('Time [s]')
                        ylabel('$\tau_l$ [Nm]', 'Interpreter', 'latex')

                        figure(3*fig_number-2); 
                        hold on; grid on;
                        plot(t,u(3,:), 'DisplayName',strcat(nodes_str, ' nodes') );
                        legend
                        title(strcat('Torque pivot',{' '},obj_func, ' to',{' '}, f_state_str) )
                        xlabel('Time [s]')
                        ylabel('$\tau_p$ [Nm]', 'Interpreter', 'latex')
                    end

                    %% Collocation constraints PLOTS (from kelly)
                    %%%% Show the error in the collocation constraint between grid points:
                    
                    cc = soln(end).interp.collCst(t);
                    
                    if strcmp(show_dyn_err, 'yes')
                        if strcmp(soln(end).problem.options.method,'trapezoid') || strcmp(soln(end).problem.options.method,'hermiteSimpson')
                            % Then we can plot an estimate of the error along the trajectory
                            figure(4000); clf;
                            sgtitle(sprintf('Dynamical Error for %s', plotfn));
                            % NOTE: the following commands have only been implemented for the direct
                            % collocation(trapezoid, hermiteSimpson) methods, and will not work for
                            % chebyshev or rungeKutta methods.                           
                            zero_points = soln(end).grid.time(1:end);

                            subplot(2,3,1); hold on;
                            plot(t,cc(1,:), zero_points, zeros(length(zero_points)), 'ro', 'MarkerSize', 4)
                            ylabel('$\dot{x}$ [m/s]', 'Interpreter', 'latex')
                            xlabel('Time [s]')

                            subplot(2,3,2); hold on;
                            plot(t,cc(2,:),zero_points, zeros(length(zero_points)), 'ro', 'MarkerSize', 4)
                            title('Coll. Error')
                            ylabel('$\dot{y}$ [m/s]', 'Interpreter', 'latex')
                            xlabel('Time [s]')

                            subplot(2,3,3); hold on;
                            plot(t,cc(3,:), zero_points, zeros(length(zero_points)), 'ro', 'MarkerSize', 4)
                            xlabel('Time [s]')
                            ylabel('$\dot{\alpha}$ [rad/s]', 'Interpreter', 'latex')

                            idx = 1:size(soln(end).info.error,2);
                            subplot(2,3,4); hold on;
                            plot(idx,soln(end).info.error(1,:),'ko', 'MarkerSize', 4);
                            xlabel('segment index')
                            ylabel('$x$ [m]', 'Interpreter', 'latex')

                            subplot(2,3,5); hold on;
                            plot(idx,soln(end).info.error(2,:),'ko', 'MarkerSize', 4);
                            title('State Error')
                            xlabel('segment index')
                            ylabel('$y$ [m]', 'Interpreter', 'latex')

                            subplot(2,3,6); hold on;
                            plot(idx,soln(end).info.error(3,:),'ko', 'MarkerSize', 4);
                            xlabel('segment index')
                            ylabel('$\alpha$ [rad]', 'Interpreter', 'latex')

                            saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'CollCst_error_',obj_func, 'to_', f_state_str,'_',nodes_str,'_nodes.png'));
                        end
                    end
                    %% Trajectory Error
                    % metric used to evaluate trajectory error from OptTraj result
                    %  compare OptTraj state with 'real' state from ode45
                    %  using interpolqted control from OptTRaj

                    if strcmp(show_traj_err,'yes') 
                        % find traj error using ode45
                        timestep = 0.001;
                        n_points = size(soln(end).grid.state, 2);
                        tspan = soln(end).grid.time(1) : timestep : soln(end).grid.time(end); % from 0 to tmax with time step h
                        dydt = @(t,y) eqns4ode(t,y, soln, sm);
                        [t_real,y_real] = ode45(dydt, tspan, soln(end).grid.state(:,1));
                        y_real = y_real.';
                        traj_err = zeros(8, n_points);

                        for i = 1:n_points
                            [~,idx] = min(abs(t_real - soln(end).grid.time(i)));
                            traj_err(:,i) = abs(soln(end).grid.state(:,i) - y_real(:, idx));
                        end

                        % plot 
                        figure(8004); clf;
                        sgtitle(strcat('Trajectory Error for Z state',{' '}, obj_func, ' obj func to', {' '}, f_state_str,{' '}, nodes_str, ' nodes'));
                        subplot(2,3,1); hold on;
                        plot(soln(end).grid.time,traj_err(1,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$x$ [m]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,2); hold on;
                        plot(soln(end).grid.time,traj_err(2,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$y$ [m]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,3); hold on;
                        plot(soln(end).grid.time,traj_err(3,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$\alpha$ [rad]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,4); hold on;
                        plot(soln(end).grid.time,traj_err(4,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$\dot{x}$ [m/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,5); hold on;
                        plot(soln(end).grid.time,traj_err(5,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$\dot{y}$ [m/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,6); hold on;
                        plot(soln(end).grid.time,traj_err(6,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$\dot{\alpha}$ [rad/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')
                        
                        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Trajectory-Error_',obj_func, 'to_', f_state_str,'_',nodes_str,'_nodes.png'));
                    end
                    
                    %% Plot OptTraj and y_real
                    
                    if strcmp(show_traj_comp,'yes') && strcmp(show_traj_err,'yes') 

                        figure(8009); clf;
                        sgtitle(strcat('Trajectory Comparaison for Z state',{' '}, obj_func, ' obj func to', {' '}, f_state_str,{' '}, nodes_str, ' nodes'));
                        subplot(2,3,1); hold on;
                        plot(soln(end).grid.time,soln(end).grid.state(1,:),tspan, y_real(1,:))
                        grid on;
                        ylabel('$x$ [m]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,2); hold on;
                        plot(soln(end).grid.time,soln(end).grid.state(2,:),tspan, y_real(2,:))
                        grid on;
                        ylabel('$y$ [m]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,3); hold on;
                        plot(soln(end).grid.time,soln(end).grid.state(3,:),tspan, y_real(3,:))
                        grid on;
                        ylabel('$\alpha$ [rad]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,4); hold on;
                        plot(soln(end).grid.time,soln(end).grid.state(4,:),tspan, y_real(4,:))
                        grid on;
                        ylabel('$\dot{x}$ [m/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,5); hold on;
                        plot(soln(end).grid.time,soln(end).grid.state(5,:),tspan, y_real(5,:))
                        grid on;
                        ylabel('$\dot{y}$ [m/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,6); hold on;
                        plot(soln(end).grid.time,soln(end).grid.state(6,:),tspan, y_real(6,:))
                        grid on;
                        ylabel('$\dot{\alpha}$ [rad/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')
                   
                        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Traj-comparaison_',obj_func, 'to_', f_state_str,'_',nodes_str,'_nodes.png'));
                        
                        figure(8010); clf;
                        sgtitle(strcat('Control Comparaison for Z state',{' '}, obj_func, ' obj func to', {' '}, f_state_str,{' '}, nodes_str, ' nodes'));
                        subplot(1,3,1); hold on;
                        plot(soln(end).grid.time,soln(end).grid.control(1,:),'ro', t, u(1,:))
                        grid on;
                        ylabel('$\tau_r$ [Nm]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(1,3,2); hold on;
                        plot(soln(end).grid.time,soln(end).grid.control(2,:),'ro',t, u(2,:))
                        grid on;
                        ylabel('$\tau_l$ [Nm]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(1,3,3); hold on;
                        plot(soln(end).grid.time,soln(end).grid.control(3,:),'ro',t, u(3,:))
                        grid on;
                        ylabel('$\tau_p$ [Nm]', 'Interpreter', 'latex')
                        xlabel('Time [s]')
                        
                        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Control-comparaison_',obj_func, 'to_', f_state_str,'_',nodes_str,'_nodes.png'));
                    end
                    
                    %% Control Effort 
                    % metric used to evaluate impact of CTC on Opttrqj result
                    %  compare OptTraj state with 'real' state from ode45
                    %  evaluated betweeen each grid point

                    if strcmp(show_ctrl_eff,'yes') 
                        % find 'real' traj using ode45, start from previosu
                        % grid point every time
                        n_points = size(soln(end).grid.state, 2);
                        ctrl_eff = zeros(8, n_points);
                        for i = 1:n_points-1
                            timestep = 0.001;
                            tspan = soln(end).grid.time(i) : timestep : soln(end).grid.time(i+1); % from 0 to tmax with time step h
                            dydt = @(t,y) eqns4ode(t,y, soln, sm);
                            [t_real,y_real] = ode45(dydt, tspan, soln(end).grid.state(:,i));
                            y_real = y_real.';
                            ctrl_eff(:,i+1) = abs(soln(end).grid.state(:,i+1) - y_real(:, end));
                        end

                        % plot 
                        figure(8005); clf;
                        sgtitle(strcat('Control Effort for Z state',{' '}, obj_func, ' obj func to', {' '}, f_state_str,{' '}, nodes_str, ' nodes'));
                        subplot(2,3,1); hold on;
                        plot(soln(end).grid.time,ctrl_eff(1,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$x$ [m]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,2); hold on;
                        plot(soln(end).grid.time,ctrl_eff(2,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$y$ [m]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,3); hold on;
                        plot(soln(end).grid.time,ctrl_eff(3,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$\alpha$ [rad]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,4); hold on;
                        plot(soln(end).grid.time,ctrl_eff(4,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$\dot{x}$ [m/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,5); hold on;
                        plot(soln(end).grid.time,ctrl_eff(5,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$\dot{y}$ [m/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')

                        subplot(2,3,6); hold on;
                        plot(soln(end).grid.time,ctrl_eff(6,:), '-o', 'MarkerSize', 3)
                        grid on;
                        ylabel('$\dot{\alpha}$ [rad/s]', 'Interpreter', 'latex')
                        xlabel('Time [s]')
                           
                        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Control-Effort_',obj_func, 'to_', f_state_str,'_',nodes_str,'_nodes.png'));
                    end
                    
                    %% save maximum dynamical error to array for plotting later
                    for n = 1:8
                        max_dyn_err(n,find(nodes==n_nodes),find(methods == method)) = max(abs(cc(n,:)));
                        avg_dyn_err(n,find(nodes==n_nodes),find(methods == method)) = mean(abs(cc(n,:)));
                        std_dyn_err(n,find(nodes==n_nodes),find(methods == method)) = std(cc(n,:));
                    end

                    %% Path Contraints plots

                    % plot path constraint varaible for last trajectory
                    if strcmp(show_path_cstr,'yes')
                        [CaP, ~] = pathCst(soln(end).grid.time, soln(end).grid.state, soln(end).grid.control, sm, XYbound);
                        leg_tb = {'X bound'; 'Y bound';'max tau(phi_{r})';'max tau(phi_{l})';'max tau(phi_{p})';'min tau(phi_{r})';'min tau(phi_{l})';'min tau(phi_{p})'};
                        figure(8000); clf;
                        for row = 1:2
                            hold on; grid on;
                            plot(soln(end).grid.time,CaP(row,:), 'DisplayName',leg_tb{row}); 
                            legend('location', 'best');
                            title(strcat('Path Constraint for each node point') )
                            xlabel('Time [s]')
                            ylabel('Path Constraints [m^2]')
                        end

                        for row = 3:8
                            figure(8000); 
                            hold on;
                            plot(soln(end).grid.time,CaP(row,:), 'DisplayName',leg_tb{row}); 
                            legend('location', 'best');
                            title(strcat('Path Constraint for each node point') )
                            xlabel('Time [s]')
                            ylabel('Path Constraints [m^2]')
                        end

                        for row = 9:size(CaP,1)
                            figure(8000); 
                            hold on; grid on;
                            plot(soln(end).grid.time,CaP(row,:), 'DisplayName',strcat('Obstacle #', sprintf('%.0f',row-2) )); 
                            legend('location', 'best');
                            title(strcat('Path Constraint for each node point') )
                            xlabel('Time [s]')
                            ylabel('Path Constraints [m^2]')
                        end

                        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Path-Constraints_',obj_func, 'to_', f_state_str,'_',nodes_str,'_nodes.png'));
                    end

                    %% make video
                    if strcmp(make_video,'yes')
                        h = soln(end).grid.time(end)/1500;
                        % convert z back to x format for video
                        K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1));
                        x = sm.psi(K_0, z(3,:), z(6,:), z(7,:), z(8,:), z(1,:), z(4,:), z(2,:), z(5,:));
                        initial_guess = soln(end).problem.guess.state;
                        video_sim(t.', x.', plotfn, h, XYbound, initial_guess);
                    end

                %end
            end

            if strcmp(show_torques, 'yes')
                 saveas(3*fig_number, strcat('Otbot/Results/Otbot/plots/',loop_name,'Tau_r_',obj_func, 'to_', f_state_str,'_', sprintf('%.0f',nodes(1)),'-',nodes_str,'_nodes.png') )
                 saveas(3*fig_number-1, strcat('Otbot/Results/Otbot/plots/',loop_name,'Tau_l_',obj_func, 'to_', f_state_str,'_', sprintf('%.0f',nodes(1)),'-',nodes_str,'_nodes.png') )
                 saveas(3*fig_number-2, strcat('Otbot/Results/Otbot/plots/',loop_name,'Tau_p_',obj_func, 'to_', f_state_str,'_', sprintf('%.0f',nodes(1)),'-',nodes_str,'_nodes.png') )
                 clear figure
                 hold off;
                 hold off;
                 hold off;
            end
        end
    end
end

% make plots comparing X for various number of nodes and methods
if strcmp(make_comp_time_plots,'yes')
    if(size(final_states,2) == 1 && length(obj_funcs) == 1)
        for i = 1:length(methods) % Computation Time plot for eahc method
                figure(9000); hold on;
                plot(nodes, time_measures(i,:), '-x', 'DisplayName', methods(i));  
                legend('location', 'best');
                grid on;
                title(strcat('Computation time for', {' '},obj_func, ' obj func to',{' '}, f_state_str) )
                xlabel('Node Points')
                ylabel('Computation Time [s]')
        end
        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Computation-time_',obj_func, 'to_', f_state_str,'_',method,'-',sprintf('%.0f',nodes(1)),'-',nodes_str,'_nodes.png'));
    else
        print("Too many objective functions or final states given, please reduce to only 1 to plot computation time comparaison");
    end
end

%Make plots to compare Dyn error for various number of nodes and methods 
if strcmp(make_dyn_err_plots,'yes')
    if(size(final_states,2) == 1 && length(obj_funcs) == 1)
        % Max Dyn error
        for i = 1:length(methods)
            figure(9002);
            %sgtitle('Maximum Dynamical Error');
            subplot(3,3,1); hold on;
            plot(nodes,max_dyn_err(1,:,i), '-x' , 'DisplayName', methods(i))
            legend('position', [0.8,0.2,0,0]);
            grid on;
            ylabel('$\dot{x} \; [m/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,2); hold on;
            plot(nodes,max_dyn_err(2,:,i), '-x' )
            grid on;
            ylabel('$\dot{y} \; [m/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,3); hold on;
            plot(nodes,max_dyn_err(3,:,i), '-x' )
            grid on;
            ylabel('$\dot{\alpha} \;[rad/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,4); hold on;
            plot(nodes,max_dyn_err(4,:,i), '-x' )
            grid on;
            ylabel('$\ddot{x} \; [m/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,5); hold on;
            plot(nodes,max_dyn_err(5,:,i), '-x' )
            grid on;
            ylabel('$\ddot{y} \; [m/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,6); hold on;
            plot(nodes,max_dyn_err(6,:,i), '-x' )
            grid on;
            ylabel('$\ddot{\alpha} \; [rad/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,7); hold on;
            plot(nodes,max_dyn_err(7,:,i), '-x' )
            grid on;
            ylabel('$\dot{\phi_{p}} \; [rad/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,8); hold on;
            plot(nodes,max_dyn_err(8,:,i), '-x' )
            grid on;
            ylabel('$\dot{\phi_{r}} \; [rad/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')   
        end
        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Max-dyn-error_',obj_func, 'to_', f_state_str,'_',method,'-',sprintf('%.0f',nodes(1)),'-',nodes_str,'_nodes.png'));

        % Avg Dyn error
        for i = 1:length(methods)
            figure(9003);
            %sgtitle('Average Dynamical Error');
            subplot(3,3,1); hold on;
            plot(nodes,avg_dyn_err(1,:,i), '-x' , 'DisplayName', methods(i))
            legend('position', [0.8,0.2,0,0]);
            grid on;
            ylabel('$\dot{x} \; [m/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,2); hold on;
            plot(nodes,avg_dyn_err(2,:,i), '-x' )
            grid on;
            ylabel('$\dot{y} \; [m/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,3); hold on;
            plot(nodes,avg_dyn_err(3,:,i), '-x' )
            grid on;
            ylabel('$\dot{\alpha} \; [rad/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,4); hold on;
            plot(nodes,avg_dyn_err(4,:,i), '-x' )
            grid on;
            ylabel('$\ddot{x} \; [m/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,5); hold on;
            plot(nodes,avg_dyn_err(5,:,i), '-x' )
            grid on;
            ylabel('$\ddot{y} \; [m/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,6); hold on;
            plot(nodes,avg_dyn_err(6,:,i), '-x' )
            grid on;
            ylabel('$\ddot{\alpha} \; [rad/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,7); hold on;
            plot(nodes,avg_dyn_err(7,:,i), '-x' )
            grid on; 
            ylabel('$\dot{\phi_p} \; [rad/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,8); hold on;
            plot(nodes,avg_dyn_err(8,:,i), '-x' )
            grid on;
            ylabel('$\dot{\phi_r} \; [rad/s]$', 'Interpreter', 'latex')
            xlabel('Node Points') 
        end    
        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Avg-dyn-error_',obj_func, 'to_', f_state_str,'_',method,'-',sprintf('%.0f',nodes(1)),'-',nodes_str,'_nodes.png'));

        % Std Dyn error
        for i = 1:length(methods)
            figure(9004);
            %sgtitle('Standard Dev. for Dynamical Error');
            subplot(3,3,1); hold on;
            plot(nodes,std_dyn_err(1,:,i), '-x' , 'DisplayName', methods(i))
            legend('position', [0.8,0.2,0,0]);
            grid on;
            ylabel('$\dot{x} \; [m/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,2); hold on;
            plot(nodes,std_dyn_err(2,:,i), '-x' )
            grid on;
            ylabel('$\dot{y} \; [m/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,3); hold on;
            plot(nodes,std_dyn_err(3,:,i), '-x' )
            grid on;
            ylabel('$\dot{\alpha} \; [rad/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,4); hold on;
            plot(nodes,std_dyn_err(4,:,i), '-x' )
            grid on;
            ylabel('$\ddot{x} \; [m/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,5); hold on;
            plot(nodes,std_dyn_err(5,:,i), '-x' )
            grid on;
            ylabel('$\ddot{y} \; [m/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,6); hold on;
            plot(nodes,std_dyn_err(6,:,i), '-x' )
            grid on;
            ylabel('$\ddot{\alpha} \; [rad/s^2]$', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,7); hold on;
            plot(nodes,std_dyn_err(7,:,i), '-x' )
            grid on;
            ylabel('$\dot{\phi_p} \; [rad/s$]', 'Interpreter', 'latex')
            xlabel('Node Points')

            subplot(3,3,8); hold on;
            plot(nodes,std_dyn_err(8,:,i), '-x' )
            grid on;
            ylabel('$\dot{\phi_r} \; [rad/s]$', 'Interpreter', 'latex')
            xlabel('Node Points')  
        end    
        saveas(gcf,strcat('Otbot/Results/Otbot/plots/',loop_name,'Std-dyn-error_',obj_func, 'to_', f_state_str,'_',method,'-',sprintf('%.0f',nodes(1)),'-',nodes_str,'_nodes.png'));
    else
        print("Too many objective functions or final states given, please reduce to only 1 to plot Dynamicla Error comparaison");
    end
end
 

