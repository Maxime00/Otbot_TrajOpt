% Resutls.m

% Function to display results from trajectory optimizaiton software
% 
% to be run from optTraj-master

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Load Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

clc; clear; close all;
load("m_struc") % structure with mass parameters
load("sm_struc") % structure with dynamics equations
addpath ../../

save_folder = 'Otbot/Results/Otbot/plots/';

XYbound =[11,11];

files_to_display = dir('Otbot/Results/Otbot/101_*.mat');

plot_all = 'no';
make_video = 'yes';
plot_init_last = 'no';
plot_kin_err = 'no';
plot_pathCstr = 'no';
plot_dyn_err = 'no';
plot_traj_err = 'no';
plot_p_state = 'yes';
plot_phi_dot_state = 'no';
plot_torques = 'yes';
plot_dyn_comp = 'no';
plot_comp_time ='no';
                
% NEED TO SET THIS DEPENDING ON DESIRED COMPARAISON PLOTS
obj_comp = 'no'; % yes"time_","force-squared_"
methods = ["trapezoid","hermiteSimpson"];%,"trapGrad","hermiteGrad","force-derivative_"  "time_","force-squared_","time-forceterm-phi-p_" ];%, , "hermiteSimpGrad"];, "trapGrad"
nodes  = [8,16,24,32,40,48]; % 


% arrays for comparaison plots 
time_measures = zeros(length(methods),length(nodes));
time_measures(end,end) = NaN;
time_measures(end-1,end) = NaN;
max_dyn_err = zeros(8,length(nodes),length(methods));
avg_dyn_err = zeros(8,length(nodes),length(methods));
std_dyn_err = zeros(8,length(nodes),length(methods));

legend_font_size = 17;
legend_text_font_size = 15;

for file_number = 1:length(files_to_display)

    %%% Load solution for display
    %fn = files_to_display{file_number};
    
    %alternative, to use with dir() command
    fn = files_to_display(file_number).name;
    load(fn) 
    
    % Get name for saving figure
    [folder, baseFileNameNoExt, extension] = fileparts(fn);
    basefn = strrep(baseFileNameNoExt, '_', ' ');
    baseFileNameNoExt = strrep(baseFileNameNoExt, '.', '');
    fileID = fopen(strcat('Otbot/Results/Otbot/info/',baseFileNameNoExt,'.txt'),'w');
    
    
    if strcmp(plot_all,'yes')
        
        %%%% Print info 
        fprintf(fileID,'File : %s \n', baseFileNameNoExt);
%         fprintf('Objective function : %s \n \n', func2str(soln(1).problem.func.pathObj));
        
        for i = (1:length(soln)) % print for each iteration
            fprintf(fileID,'Method for iter %.0f : %s \t', i, soln(i).problem.options.method);
            fprintf(fileID,'%s accuracy \t', soln(i).problem.options.defaultAccuracy);
            if strcmp(soln(i).problem.options.method,'trapezoid')
                fprintf(fileID,'%.0f segments\t \t \t', soln(i).problem.options.trapezoid.nGrid);
            elseif strcmp(soln(i).problem.options.method,'trapezoid')
                fprintf(fileID,'%.0f segments\t \t \t', soln(i).problem.options.trapezoid.nSegment);
            elseif strcmp(soln(i).problem.options.method,'chebyshev')
                fprintf(fileID,'%.0f collocation points \t \t \t', soln(i).problem.options.chebyshev.nColPts);
            elseif strcmp(soln(i).problem.options.method,'rungeKutta')
                fprintf(fileID,'%.0f segments, %.0f subSteps \t \t \t', soln(i).problem.options.rungeKutta.nSegment, soln(i).problem.options.rungeKutta.nSubStep);
            end
            
            fprintf(fileID,'Final objVal : %f \n', soln(i).info.objVal);
        end
        fprintf(fileID,'\n');
        fclose(fileID);
        %%%% Plots:
        
        % Show the trajectory, torque and objective function
        figure(2*file_number -1);  clf;
        sgtitle(sprintf('%s iter 1-%.0f', basefn ,length(soln)));

        % Plot guess trajectory on first line
        subplot(length(soln)+1,2,1); hold on;
        plot(soln(1).problem.guess.time,soln(1).problem.guess.state(1,:),'-o',soln(1).problem.guess.time,soln(1).problem.guess.state(2,:),'-.', soln(1).problem.guess.time,soln(1).problem.guess.state(3,:),'.');
        legend('x', 'y', 'alpha')
        title('Original trajectory of Otbot vs time')
        xlabel('Time [s]')
        ylabel('Coordinates [m]')

        subplot(length(soln)+1,2,2); hold on;
        plot(soln(1).problem.guess.time,soln(1).problem.guess.control(1,:),'-o',soln(1).problem.guess.time,soln(1).problem.guess.control(2,:),'-.', soln(1).problem.guess.time,soln(1).problem.guess.control(3,:),'.');
        legend('$\tau_r$', '$\tau_l$', '$\tau_p$', 'Interpreter', 'latex')
        title('Original torques of Otbot vs time')
        xlabel('Time [s]')
        ylabel('Torques [Nm]')

        %%% Loop to display each mesh refinement step 
        for i = (1:length(soln))
           %%%% Unpack the simulation
            t = linspace(soln(i).grid.time(1), soln(i).grid.time(end), 1500);
            z = soln(i).interp.state(t);
            u = soln(i).interp.control(t);

            subplot(length(soln)+1,2,2*i+1); hold on;
            plot(t,z(1,:),'-o',t,z(2,:),'-.', t,z(3,:),'.');
            legend('x', 'y', 'alpha')
            title(sprintf('Trajectory iter %.0f', i))
            xlabel('Time [s]')
            ylabel('Coordinates [m]')

            subplot(length(soln)+1,2,2*i+2); hold on;
            plot(t,u(1,:),'-o',t,u(2,:),'-.', t,u(3,:),'.');
            legend('t_r', 't_l', 't_p')
            title(sprintf('Torques iter %.0f', i))
            xlabel('Time [s]')
            ylabel('Torques [Nm]')
        end
        
        saveas(gcf, strcat(save_folder,baseFileNameNoExt,'_all_iter'))

        %%%% Show the error in the collocation constraint between grid points:
        for i = (1:length(soln))
            if strcmp(soln(end).problem.options.method,'trapezoid') || strcmp(soln(end).problem.options.method,'trapezoid')
                % Then we can plot an estimate of the error along the trajectory
                figure(100*i+ 2*file_number ); clf;
                sgtitle(sprintf('Collocation Constraints for iter %.0f', i));
                % NOTE: the following commands have only been implemented for the direct
                % collocation(trapezoid, trapezoid) methods, and will not work for
                % chebyshev or rungeKutta methods.
                cc = soln(i).interp.collCst(t);
                zero_points = soln(i).grid.time(1:2:end);

                subplot(2,3,1); hold on;
                plot(t,cc(1,:), zero_points, zeros(length(zero_points)), 'ro' )
                ylabel('d/dt Otbot x position [m/s]')
                xlabel('time [s]')

                subplot(2,3,2); hold on;
                plot(t,cc(2,:),zero_points, zeros(length(zero_points)), 'ro')
                title('Collocation Error:   dx/dt - f(t,x,u)')
                ylabel('d/dt Otbot y position [m/s]')
                xlabel('time [s]')

                subplot(2,3,3); hold on;
                plot(t,cc(3,:), zero_points, zeros(length(zero_points)), 'ro')
                xlabel('time [s]')
                ylabel('d/dt alpha angle [rad/s]')
                   
                idx = 1:size(soln(i).info.error,2);
                subplot(2,3,4); hold on;
                plot(idx,soln(i).info.error(1,:),'ko');
                xlabel('segment index')
                ylabel('Otbot x position [m]')

                subplot(2,3,5); hold on;
                plot(idx,soln(i).info.error(2,:),'ko');
                title('State Error')
                xlabel('segment index')
                ylabel('Otbot y position [m]')

                subplot(2,3,6); hold on;
                plot(idx,soln(i).info.error(3,:),'ko');
                xlabel('segment index')
                ylabel('alpha angle [rad]');
            end
        
        saveas(gcf, strcat(save_folder,baseFileNameNoExt,sprintf('_collCst_iter_%.0f', i)))
        end
        
    else   % Plot only guess and last solution found
        
        %%%% Print info 
        fprintf(fileID,'File : %s \n', baseFileNameNoExt);
        %fprintf('Objective function : %s \n \n', func2str(soln(1).problem.func.pathObj));
        
        for i = (1:length(soln)) % print for each iteration
            fprintf(fileID,'Method for iter %.0f : %s \t', i, soln(i).problem.options.method);
            fprintf(fileID,'%s accuracy \t', soln(i).problem.options.defaultAccuracy);
            if strcmp(soln(i).problem.options.method,'trapezoid')
                fprintf(fileID,'%.0f segments\t \t \t', soln(i).problem.options.trapezoid.nGrid);
            elseif strcmp(soln(i).problem.options.method,'trapezoid')
                fprintf(fileID,'%.0f segments\t \t \t', soln(i).problem.options.trapezoid.nSegment);
            elseif strcmp(soln(i).problem.options.method,'chebyshev')
                fprintf(fileID,'%.0f segments\t \t \t', soln(i).problem.options.chebyshev.nColPts);
            elseif strcmp(soln(i).problem.options.method,'rungeKutta')
                fprintf(fileID,'%.0f segments, %.0f subSteps \t \t \t', soln(i).problem.options.rungeKutta.nSegment, soln(i).problem.options.rungeKutta.nSubStep);
            end
            fprintf(fileID,'Final objVal : %f \n ', soln(i).info.objVal); 
            fprintf(fileID,'Final traj timestep : %f \n ', soln(i).grid.time(2));
            fprintf(fileID,'Computation Time : %f \n ', soln(i).info.nlpTime);
        end
        fprintf(fileID,'\n');
        fclose(fileID);

        %%%% Unpack the simulation
        t = linspace(soln(end).grid.time(1), soln(end).grid.time(end), 1500);
        z = soln(end).interp.state(t);
        u = soln(end).interp.control(t);
        node_times = soln(end).grid.time(1:end);
        node_states = soln(end).grid.state(:, 1:end);
        node_control = soln(end).grid.control(:, 1:end);
        
        
        % Show the trajectory, torque and objective function
        if strcmp(plot_init_last, 'yes')
            figure(3*file_number -2);  clf;
            sgtitle(sprintf('%s guess and last iter', basefn ));

            % Plot guess trajectory on first line
            subplot(2,2,1); hold on;
            plot(soln(1).problem.guess.time,soln(1).problem.guess.state(1,:),'-o',soln(1).problem.guess.time,soln(1).problem.guess.state(2,:),'-.', soln(1).problem.guess.time,soln(1).problem.guess.state(3,:),'.');
            legend('$x$', '$y$', '$\alpha$', 'Interpreter', 'latex')
            title('Original trajectory of Otbot vs time')
            xlabel('Time [s]')
            ylabel('Coordinates [m]')

            subplot(2,2,2); hold on;
            plot(soln(1).problem.guess.time,soln(1).problem.guess.control(1,:),'-o',soln(1).problem.guess.time,soln(1).problem.guess.control(2,:),'-.', soln(1).problem.guess.time,soln(1).problem.guess.control(3,:),'.');
            legend('$\tau_r$', '$\tau_l$', '$\tau_p$', 'Interpreter', 'latex')
            title('Original torques of Otbot vs time')
            xlabel('Time [s]')
            ylabel('Torques [Nm]')



            % Plot last found trajectory on second line 
            subplot(2,2,3); hold on;
            p = plot(t,z(1,:),t,z(2,:), t,z(3,:), node_times, node_states(1,:), 'ro', node_times, node_states(2,:), 'ro', node_times, node_states(3,:), 'ro' );
            p(4).MarkerSize = 3;
            p(5).MarkerSize = 3;
            p(6).MarkerSize = 3;   
            legend('$x$', '$y$', '$\alpha$', 'Interpreter', 'latex')
            title(sprintf('Trajectory iter %.0f', length(soln)))
            xlabel('Time [s]')
            ylabel('Coordinates [m]')

            subplot(2,2,4); hold on;
            pp= plot(t,u(1,:),t,u(2,:), t,u(3,:),node_times, node_control(1,:), 'ro', node_times, node_control(2,:), 'ro', node_times, node_control(3,:), 'ro' );
            pp(4).MarkerSize = 3;
            pp(5).MarkerSize = 3;
            pp(6).MarkerSize = 3;   
            legend('$\tau_r$', '$\tau_l$', '$\tau_p$', 'Interpreter', 'latex')
            title(sprintf('Torques iter %.0f', length(soln)))
            xlabel('Time [s]')
            ylabel('Torques [Nm]')
            saveas(gcf, strcat(save_folder,baseFileNameNoExt,'_last_iter'))
        end
        
        if strcmp(plot_p_state, 'yes')
            figure(8000); clf;
            p = plot(t,z(1,:),t,z(2,:), t,z(3,:), node_times, node_states(1,:), 'ro', node_times, node_states(2,:), 'ro', node_times, node_states(3,:), 'ro' );
            p(4).MarkerSize = 3.5;
            p(5).MarkerSize = 3.5;
            p(6).MarkerSize = 3.5;
            grid on;
            legend('$x$', '$y$', '$\alpha$', 'Interpreter', 'latex', 'FontSize',12,'Location', 'best')
            %title('P-state Variables')
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            ylabel('Coordinates $[m] [rad]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            set(gcf, 'Position', [10 10 650 600])
            saveas(gcf, strcat(save_folder,baseFileNameNoExt,'_p_state','.eps'), 'epsc')
        end
        
        if strcmp(plot_phi_dot_state, 'yes')
            
            if contains(fn, 'X-model')
                K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1));
                x = z;
            else % Z model, convert z back to x format for video
                K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1));
                x = sm.psi(K_0, z(3,:), z(6,:), z(7,:), z(8,:), z(1,:), z(4,:), z(2,:), z(5,:));
                node_phi_states = sm.psi(K_0, node_states(3,:), node_states(6,:), node_states(7,:), node_states(8,:), node_states(1,:), node_states(4,:), node_states(2,:), node_states(5,:));
            end  
            
            figure(8000); clf;
            p = plot(t,x(10,:),t,x(11,:), t,x(12,:), node_times, node_phi_states(10,:), 'ro', node_times, node_phi_states(11,:), 'ro', node_times, node_phi_states(12,:), 'ro' );
            p(4).MarkerSize = 3.5;
            p(5).MarkerSize = 3.5;
            p(6).MarkerSize = 3.5;
            grid on;
            legend('$\dot{\varphi}_r$', '$\dot{\varphi}_l$', '$\dot{\varphi}_p$', 'Interpreter', 'latex', 'FontSize',12,'Location', 'best')
            %title('P-state Variables')
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            ylabel('Coordinates $[rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            set(gcf, 'Position', [10 10 650 600])
            saveas(gcf, strcat(save_folder,baseFileNameNoExt,'_phi-dot_state','.eps'), 'epsc')
        end
        
        if strcmp(plot_torques, 'yes')
            figure(8001); clf;
            pp= plot(t,u(1,:),t,u(2,:),t,u(3,:), node_times, node_control(1,:), 'ro', node_times, node_control(2,:), 'ro', node_times, node_control(3,:), 'ro' );
            %pp= plot(t,u(1,:),t,u(2,:), node_times, node_control(1,:), 'ro', node_times, node_control(2,:), 'ro');
            pp(4).MarkerSize = 3.5;
            pp(5).MarkerSize = 3.5;
            pp(6).MarkerSize = 3.5;
            %axis([0 10 -80 80])
            grid on; 
            set(gcf, 'Position', [10 10 650 600])%1000 320
            legend('$\tau_r$', '$\tau_l$','$\tau_p$', 'Interpreter', 'latex', 'FontSize',12, 'location', 'best')%
            %title('Torques Profile')
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            ylabel('Torques $[Nm]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            saveas(gcf, strcat(save_folder,baseFileNameNoExt,'_torques','.eps'), 'epsc')
        end
        
        %%% Calculate kinematic error to measure realism of solution
        if strcmp(plot_kin_err, 'yes')
            cstr_1 = zeros(3,length(t));

            % convert z back to x format for kin constraints
            if contains(fn, 'X-model')
                K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1));
                x = z;
            else % Z model, convert z back to x format for video
                K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1));
                x = sm.psi(K_0, z(3,:), z(6,:), z(7,:), z(8,:), z(1,:), z(4,:), z(2,:), z(5,:));
            end   

            for k = 1:length(t)
                cstr_1(:,k) = sm.newJmatrix(x(3,k), x(6,k)) * x(7:12,k);
            end
            %  cstr_2 = r/(2*l_2) *(phi_r(t) -phi_r(0)) -r/(2*l_2) *(phi_l(t) -phi_l(0)) + (phi_p(t) - phi_p(0)) -(alpha(t) - alpha(0) )
            %cstr_2 = m.r/(2*m.l_2) *(x(4,:) - x(4,1) + x(5,1) - x(5,:)) +(x(6,:) - x(6,1)) -(x(3,:) - x(3,1));
            cstr_2 = sm.K(z(3,:), x(5,:), z(7,:), z(8,:)) + K_0;

            figure(3*file_number -1); clf;
            plot(t,cstr_1(1,:), t, cstr_1(2,:), t, cstr_1(3,:), t, cstr_2);  
            legend('$F_1 \; [m/s]$', '$F_2 \; [m/s]$','$F_3 \; [rad/s]$','$F_4 \; [rad]$', 'Interpreter', 'latex', 'FontSize',12, 'Location', 'best');
            %title('Kinematic Error')
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            ylabel('Kinematic errors', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            set(gcf, 'Position', [10 10 650 600])
            grid on;
            saveas(gcf, strcat(save_folder,baseFileNameNoExt,'_kin_error','.eps'), 'epsc');
        end
        
        %%%% Show the error in the collocation constraint between grid points:
        if strcmp(soln(end).problem.options.method,'trapezoid') || strcmp(soln(end).problem.options.method,'hermiteSimpson')
            % Then we can plot an estimate of the error along the trajectory
            
            %sgtitle(sprintf('%s Collocation Constraints error', basefn));
            % NOTE: the following commands have only been implemented for the direct
            % collocation(trapezoid, trapezoid) methods, and will not work for
            % chebyshev or rungeKutta methods.
            cc = soln(end).interp.collCst(t);
            zero_points = soln(end).grid.time(1:end);
            
            if strcmp(plot_dyn_err, 'yes')
                figure(3*file_number ); clf;
                subplot(1,3,1); hold on;
                plot(t,cc(1,:), zero_points, zeros(length(zero_points)), 'ro', 'MarkerSize', 4)
                ylabel('$\dot{x} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
                xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

                subplot(1,3,2); hold on;
                plot(t,cc(2,:),zero_points, zeros(length(zero_points)), 'ro', 'MarkerSize', 4)
                %title('Coll. Error')
                ylabel('$\dot{y} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
                xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

                subplot(1,3,3); hold on;
                plot(t,cc(3,:), zero_points, zeros(length(zero_points)), 'ro', 'MarkerSize', 4)
                xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
                ylabel('$\dot{\alpha} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)

                set(gcf, 'Position',  [100, 100, 1500, 500])
                saveas(gcf, strcat(save_folder,baseFileNameNoExt,'_collCst_error','.eps'), 'epsc') 
            end
        end
        
        
        %% Trajectory Error
        % metric used to evaluate trajectory error from OptTraj result
        %  compare OptTraj state with 'real' state from ode45
        %  using interpolqted control from OptTRaj

        if strcmp(plot_traj_err,'yes') 
            % find traj error using ode45
            timestep = 0.001;
            n_points = size(soln(end).grid.state, 2);
            
            if contains(fn, 'X-model')
                dydt = @(t,y) eqns4odeXmodel(t,y, soln, sm);
                traj_err = zeros(12, n_points);
            else % Z model
                dydt = @(t,y) eqns4ode(t,y, soln, sm);
                traj_err = zeros(8, n_points);
            end
            
            tspan = soln(end).grid.time(1) : timestep : soln(end).grid.time(end); % from 0 to tmax with time step h
            [t_real,y_real] = ode45(dydt, tspan, soln(end).grid.state(:,1));
            y_real = y_real.';

            for i = 1:n_points
                [~,idx] = min(abs(t_real - soln(end).grid.time(i)));
                traj_err(:,i) = abs(soln(end).grid.state(:,i) - y_real(:, idx));
            end

            % plot 
            figure(8004); clf;
            %sgtitle('Trajectory Error for Z state');
            subplot(2,3,1); hold on;
            plot(soln(end).grid.time,traj_err(1,:), '-o', 'MarkerSize', 4)
            grid on;
            ylabel('$x \; [m]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,3,2); hold on;
            plot(soln(end).grid.time,traj_err(2,:), '-o', 'MarkerSize', 4)
            grid on;
            ylabel('$y \; [m]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,3,3); hold on;
            plot(soln(end).grid.time,traj_err(3,:), '-o', 'MarkerSize', 4)
            grid on;
            ylabel('$\alpha \; [rad]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,3,4); hold on;
            plot(soln(end).grid.time,traj_err(4,:), '-o', 'MarkerSize', 4)
            grid on;
            ylabel('$\dot{x} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,3,5); hold on;
            plot(soln(end).grid.time,traj_err(5,:), '-o', 'MarkerSize', 4)
            grid on;
            ylabel('$\dot{y} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,3,6); hold on;
            plot(soln(end).grid.time,traj_err(6,:), '-o', 'MarkerSize', 4)
            grid on;
            ylabel('$\dot{\alpha} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)
            
            set(gcf, 'Position',  [100, 100, 1500, 1000])
            saveas(gcf,strcat(save_folder,baseFileNameNoExt,'_traj_error','.eps'), 'epsc');
        end
        
        
        % plot path constraints
        if strcmp (plot_pathCstr, 'yes')
            [CaP, ~] = pathCst(soln(end).grid.time, soln(end).grid.state);
            for row = 1:size(CaP,1)
                figure(9000); 
                hold on; grid on;
                plot(soln(end).grid.time,CaP(row,:), 'DisplayName',strcat('Obstacle #',sprintf('%.0f',row) ) ); 
                legend
                title(strcat('Path Constraint for each node point') )
                xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize',legend_font_size)
                ylabel('Path Constraint $[m^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            end

            saveas(gcf,strcat(save_folder,baseFileNameNoExt,'_path_contraints','.png'))
        end
        
    end
    
    %% save computation time, dynamical error to array for plotting later
    % Save and print computation time 
    
    % for comparing by method
    method = soln.problem.options.method;
    if strcmp(method, 'trapezoid')
        n_nodes = soln.problem.options.trapezoid.nGrid;
        if strcmp(soln.problem.options.nlpOpt.GradObj, 'on')
            method = 'trapGrad';
        end
    else
        n_nodes = soln.problem.options.hermiteSimpson.nSegment;
        if strcmp(soln.problem.options.nlpOpt.GradObj, 'on')
            method = 'hermiteGrad';
        end
    end
    
    % for comparing by objective fucntion (method=obj_func)
    if strcmp(obj_comp, 'yes')
        if contains(fn, 'time')
            method = "time_";
        elseif contains(fn, 'force-squared')
            method = "force-squared_";
        elseif contains(fn, 'time-forceterm-phi-p')
            method = "time-forceterm-phi-p_";
        end
        baseFileNameNoExt = strcat(baseFileNameNoExt,'_obj-comp');
    end
    
    time_measures(find(methods == method),find(nodes ==n_nodes))= 0;
    for i = 1:size(soln)
        time_measures(find(methods == method),find(nodes ==n_nodes))= time_measures(find(methods == method),find(nodes ==n_nodes))+ soln(i).info.nlpTime;
    end
    
    for n = 1:8
        max_dyn_err(n,find(nodes==n_nodes),find(methods == method)) = max(abs(cc(n,:)));
        avg_dyn_err(n,find(nodes==n_nodes),find(methods == method)) = mean(abs(cc(n,:)));
        std_dyn_err(n,find(nodes==n_nodes),find(methods == method)) = std(cc(n,:));
    end
    
    
    if strcmp(make_video,'yes')
        n_inter = 1500;
        t = linspace(soln(i).grid.time(1), soln(i).grid.time(end), n_inter);
        z = soln(i).interp.state(t);
        h = soln(i).grid.time(end)/n_inter;
        
        initial_guess = soln(end).problem.guess.state;
               
        if contains(fn, 'X-model')
            x = z;
        else % Z model, convert z back to x format for video
            K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1));
            x = sm.psi(K_0, z(3,:), z(6,:), z(7,:), z(8,:), z(1,:), z(4,:), z(2,:), z(5,:));
        end   
     
        video_sim(t.', x.', baseFileNameNoExt, h, XYbound, initial_guess);
        
    end
        
end


% make plots comparing X for various number of nodes and methods
if strcmp(plot_comp_time,'yes')
    
    for i = 1:length(methods) % Computation Time plot for eahc method
            figure(9000); hold on;
            leg= strrep(methods(i), '_', ' ');
            plot(nodes, time_measures(i,:), '-x', 'DisplayName', leg);  
            legend('location', 'northwest','Interpreter', 'latex','FontSize',14 );%
            grid on;
            %title('Computation time' )
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            ylabel('Computation Time $[s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
    end
    set(gcf, 'Position',  [100, 100,  1200, 320]) %1000, 470
    saveas(gcf,strcat(save_folder,baseFileNameNoExt,'_computation-time','.eps'), 'epsc');

end

%Make plots to compare Dyn error for various number of nodes and methods 
if strcmp(plot_dyn_comp,'yes')
        % Max Dyn error
        for i = 1:length(methods)
            figure(9002);
            %sgtitle('Maximum Dynamical Error');
            subplot(3,3,1); hold on;
            leg= strrep(methods(i), '_', ' ');
            plot(nodes,max_dyn_err(1,:,i), '-x' , 'DisplayName', leg)
            legend('position', [0.8,0.2,0,0],'Interpreter', 'latex',  'FontSize',12);%
            grid on;
            ylabel('$\dot{x} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,2); hold on;
            plot(nodes,max_dyn_err(2,:,i), '-x' )
            grid on;
            ylabel('$\dot{y} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,3); hold on;
            plot(nodes,max_dyn_err(3,:,i), '-x' )
            grid on;
            ylabel('$\dot{\alpha} \;[rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,4); hold on;
            plot(nodes,max_dyn_err(4,:,i), '-x' )
            grid on;
            ylabel('$\ddot{x} \; [m/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,5); hold on;
            plot(nodes,max_dyn_err(5,:,i), '-x' )
            grid on;
            ylabel('$\ddot{y} \; [m/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,6); hold on;
            plot(nodes,max_dyn_err(6,:,i), '-x' )
            grid on;
            ylabel('$\ddot{\alpha} \; [rad/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,7); hold on;
            plot(nodes,max_dyn_err(7,:,i), '-x' )
            grid on;
            ylabel('$\dot{\varphi_{p}} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,8); hold on;
            plot(nodes,max_dyn_err(8,:,i), '-x' )
            grid on;
            ylabel('$\dot{\varphi_{r}} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)   
        end
        set(gcf, 'Position',  [100, 100, 1000, 1000])
        saveas(gcf,strcat(save_folder,baseFileNameNoExt,'_max-dyn-error','.eps'), 'epsc');

        % Avg Dyn error
        for i = 1:length(methods)
            figure(9003);
            %sgtitle('Average Dynamical Error');
            subplot(2,4,1); hold on;
            leg= strrep(methods(i), '_', ' ');
            plot(nodes,avg_dyn_err(1,:,i), '-x' , 'DisplayName', leg)
            %legend('position', [0.8,0.2,0,0], 'Interpreter', 'latex', 'FontSize',14);%
            grid on;
            ylabel('$\dot{x} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,4,2); hold on;
            plot(nodes,avg_dyn_err(2,:,i), '-x' )
            grid on;
            ylabel('$\dot{y} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,4,3); hold on;
            plot(nodes,avg_dyn_err(3,:,i), '-x' )
            grid on;
            ylabel('$\dot{\alpha} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,4,4); hold on;
            plot(nodes,avg_dyn_err(4,:,i), '-x' )
            grid on;
            ylabel('$\ddot{x} \; [m/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,4,5); hold on;
            plot(nodes,avg_dyn_err(5,:,i), '-x' )
            grid on;
            ylabel('$\ddot{y} \; [m/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,4,6); hold on;
            plot(nodes,avg_dyn_err(6,:,i), '-x' )
            grid on;
            ylabel('$\ddot{\alpha} \; [rad/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,4,7); hold on;
            plot(nodes,avg_dyn_err(7,:,i), '-x' )
            grid on; 
            ylabel('$\dot{\varphi_p} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(2,4,8); hold on;
            plot(nodes,avg_dyn_err(8,:,i), '-x' )
            grid on;
            ylabel('$\dot{\varphi_r} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)   
        end    
        set(gcf, 'Position',  [100, 100, 1200, 320])
        saveas(gcf,strcat(save_folder,baseFileNameNoExt,'_avg-dyn-error','.eps'), 'epsc');

        % Std Dyn error
        for i = 1:length(methods)
            figure(9004);
            %sgtitle('Standard Dev. for Dynamical Error');
            subplot(3,3,1); hold on;
            leg= strrep(methods(i), '_', ' ');
            plot(nodes,std_dyn_err(1,:,i), '-x' , 'DisplayName', leg)
            legend('position', [0.8,0.2,0,0], 'Interpreter', 'latex', 'FontSize',12);
            grid on;
            ylabel('$\dot{x} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,2); hold on;
            plot(nodes,std_dyn_err(2,:,i), '-x' )
            grid on;
            ylabel('$\dot{y} \; [m/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,3); hold on;
            plot(nodes,std_dyn_err(3,:,i), '-x' )
            grid on;
            ylabel('$\dot{\alpha} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,4); hold on;
            plot(nodes,std_dyn_err(4,:,i), '-x' )
            grid on;
            ylabel('$\ddot{x} \; [m/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,5); hold on;
            plot(nodes,std_dyn_err(5,:,i), '-x' )
            grid on;
            ylabel('$\ddot{y} \; [m/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,6); hold on;
            plot(nodes,std_dyn_err(6,:,i), '-x' )
            grid on;
            ylabel('$\ddot{\alpha} \; [rad/s^2]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,7); hold on;
            plot(nodes,std_dyn_err(7,:,i), '-x' )
            grid on;
            ylabel('$\dot{\varphi_p} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)

            subplot(3,3,8); hold on;
            plot(nodes,std_dyn_err(8,:,i), '-x' )
            grid on;
            ylabel('$\dot{\varphi_r} \; [rad/s]$', 'Interpreter', 'latex', 'FontSize',legend_font_size)
            xlabel('Number of knot points', 'Interpreter', 'latex', 'FontSize',legend_text_font_size)   
        end  
        set(gcf, 'Position',  [100, 100, 1000, 1000])
        saveas(gcf,strcat(save_folder,baseFileNameNoExt,'_std-dyn-error','.eps'), 'epsc');
end
 


