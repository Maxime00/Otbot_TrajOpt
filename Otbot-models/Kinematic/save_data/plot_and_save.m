function plot_and_save(sln)
% Plot trajectory and control varaibesl and save plots and solution to
% results folder

y = sln.Y;
t = sln.T;

end_traj = circle_traj(t.');
e_x = end_traj(1,:)-y(:,1).';
e_y = end_traj(2,:)-y(:,2).';
e_a = end_traj(3,:)-y(:,3).';

K = control_hyper_parameters();

fn1 = sprintf('../Results/Inertia OG/Kinematic - Simple Controller/plots/Trajectory for K_x %1.1f, K_y %1.1f, K_a %1.1f.png', K(1,1), K(2,2), K(3,3));
fn2 = sprintf('../Results/Inertia OG/Kinematic - Simple Controller/plots/Error for K_x %1.1f, K_y %1.1f, K_a %1.1f.png', K(1,1), K(2,2), K(3,3));
fn = sprintf('../Results/Inertia OG/Kinematic - Simple Controller/sln/Solution for K_x %1.1f, K_y %1.1f, K_a %1.1f.png', K(1,1), K(2,2), K(3,3));

struct2csv(sln, fn)

figure(1)
plot(t,y(:,1),'-o',t,y(:,2),'-.', t,y(:,3),'.');
legend('x', 'y', 'alpha')
title('Position and angle of Otbot over time')
xlabel('Time')
ylabel('Coordinates')
saveas(gcf, fn1)

figure(2)
plot(t,e_x,'-o',t,e_y,'-.', t,e_a,'.');
legend('e_x', 'e_y', 'e_a')
xlabel('Time')
ylabel('Error')
saveas(gcf, fn2)

end

