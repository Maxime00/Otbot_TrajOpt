function plot_and_save(sln, cd, fn, m, soln, traj_var)
% Plot trajectory and control variables and save plots and solution to
% results folder

y = sln.Y;
t = sln.T;
u = sln.U;

% calculate error depending on goal traj

switch traj_var
    case 'fixed'
        end_traj = zeros(3,size(t,1));
        end_traj(1,:) = 0;
        end_traj(2,:) = 10;
        end_traj(3,:) = 0;
        
    case 'circle'
        end_traj = circle_traj(t, m);
        
    case 'square' 
        end_traj = zeros(3,length(t));
        for i = 1:length(t)
            end_traj(:,i) = square_traj(t(i));
        end
               
    case 'M_traj'
        end_traj = zeros(3,length(t));
        for i = 1:length(t)
            [end_traj(:,i), ~] = M_traj(t(i));
        end
   
    case 'Line_traj'
        end_traj = zeros(3,length(t));
        for i = 1:length(t)
            [end_traj(:,i), ~] = Line_traj(t(i));
        end
        
    case 'TrajOpt'
        end_traj = soln.interp.state(t);       
        
end

e_x = end_traj(1,:)-y(1,:);
e_y = end_traj(2,:)-y(2,:);
e_a = end_traj(3,:)-y(3,:);

% solutino to display K gain in plot titles
%[Kp, Kv, Ki] = control_hyper_parameters();
% fn1 = sprintf('%s/plots/Trajectory for %s Kp_x %1.1f, Kp_y %1.1f, Kp_a %1.1f, Kv_x %1.1f, Kv_y %1.1f, Kv_a %1.1f, Ki_x %1.1f, Ki_y %1.1f and Ki_a %1.1f.png', cd, fn, Kp(1,1), Kp(2,2), Kp(3,3), Kv(1,1), Kv(2,2),Kv(3,3), Ki(1,1), Ki(2,2), Ki(3,3));
% fn5 = sprintf('%s/plots/Phi for %s Kp_x %1.1f, Kp_y %1.1f, Kp_a %1.1f, Kv_x %1.1f, Kv_y %1.1f, Kv_a %1.1f, Ki_x %1.1f, Ki_y %1.1f and Ki_a %1.1f.png', cd, fn, Kp(1,1), Kp(2,2), Kp(3,3), Kv(1,1), Kv(2,2),Kv(3,3), Ki(1,1), Ki(2,2), Ki(3,3));
% fn2 = sprintf('%s/plots/Torques for %s Kp_x %1.1f, Kp_y %1.1f, Kp_a %1.1f, Kv_x %1.1f, Kv_y %1.1f, Kv_a %1.1f, Ki_x %1.1f, Ki_y %1.1f and Ki_a %1.1f.png', cd, fn, Kp(1,1), Kp(2,2), Kp(3,3), Kv(1,1), Kv(2,2),Kv(3,3), Ki(1,1), Ki(2,2), Ki(3,3));
% fn3 = sprintf('%s/plots/Error for %s Kp_x %1.1f, Kp_y %1.1f, Kp_a %1.1f, Kv_x %1.1f, Kv_y %1.1f, Kv_a %1.1f, Ki_x %1.1f, Ki_y %1.1f and Ki_a %1.1f.png', cd, fn, Kp(1,1), Kp(2,2), Kp(3,3), Kv(1,1), Kv(2,2),Kv(3,3), Ki(1,1), Ki(2,2), Ki(3,3));

fn1 = sprintf('%s/plots/Trajectory_for_%s.eps', cd, fn);
fn5 = sprintf('%s/plots/Phi_for_%s.eps', cd, fn );
fn2 = sprintf('%s/plots/Torques_for_%s.eps', cd, fn);
fn3 = sprintf('%s/plots/Error_for_%s.eps', cd, fn);



fn4 = sprintf('%s/sln/sln_%s.mat', cd, fn);

save(fn4,"sln","sln")

figure(1)
plot(t,y(1,:),t,y(2,:), t,y(3,:));
legend('$x$', '$y$', '$\alpha$', 'Interpreter', 'latex', 'Location', 'best')
%title('Position and angle of Otbot over time')
xlabel('Time [s]')
ylabel('Coordinates [m][rad]')
grid on;
saveas(gcf,fn1, 'epsc')

figure(2)
plot(t,y(4,:),t,y(5,:), t,y(6,:));
legend('$\dot{x}$', '$\dot{y}$', '$\dot{\alpha}$', 'Interpreter', 'latex', 'Location', 'best')
%title('Speed coords of Otbot over time')
xlabel('Time [s]')
ylabel('Speeds [m/s][rad/s]')
grid on;
saveas(gcf, fn5, 'epsc')

figure(4)
plot(t,u(1,:),t,u(2,:), t,u(3,:));
legend('$\tau_r$', '$\tau_l$', '$\tau_p$', 'Interpreter', 'latex', 'Location', 'best')
%title('Torques over time')
xlabel('Time [s]')
ylabel('Torques [Nm]')
grid on;
saveas(gcf, fn2, 'epsc')

figure(3)
plot(t,e_x,t,e_y, t,e_a);
legend('$e_x$', '$e_y$', '$e_{\alpha}$', 'Interpreter', 'latex', 'Location', 'best')
%title('Errors over time')
xlabel('Time [s]')
ylabel('Positional Error [m][rad]')
grid on;
saveas(gcf, fn3, 'epsc')


end

