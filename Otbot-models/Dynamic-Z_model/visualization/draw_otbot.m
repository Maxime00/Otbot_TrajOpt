function [  ] = draw_otbot( m, q, soln, traj_var, end_pos, rObst_t)
%DRAW_DM Summary of this function goes here
%   Detailed explanation goes here
figure(100);
pos_fig1 = [0 0 1920 1080];
set(gcf,'Position',pos_fig1)

xs(1)=q.x;
xs(2)=q.y;
xs(3)=q.alpha;
xs(4)=q.varphi_r;
xs(5)=q.varphi_l;
xs(6)=q.varphi_p;

% Drawing axis
trplot(eye(3), 'length', 1, 'color', 'k');

hold on;
grid on;

%%%%%%%%%%%%%%%%%
view([0,90]); % this part sets the view of the plot coment if you want isometric

%%%%%%%%%%%%%%%%%
axis([-11 11 -11 11]);

xticks(-100:1:100);
yticks(-100:1:100);

% Defining rotation matrix
Rmat = m.Rzmat(xs(3)-xs(6));

% Drawing chassis Body

trvec = [xs(1);xs(2);0];

CBp = Rmat*m.CBprel + [trvec,trvec,trvec]; % Rotation + Translation

fill3(CBp(1,:),CBp(2,:),CBp(3,:),(1/255)*[191,191,191]);

% Drawing Wheels     
RWBp_r = Rmat*m.RWBprel; % Rotation

RWBp_l = RWBp_r;   % Rotation is the same for both wheels  

RWBp_r = RWBp_r + [CBp(:,2),CBp(:,2),CBp(:,2),CBp(:,2)]; % Translation 
RWBp_l = RWBp_l + [CBp(:,3),CBp(:,3),CBp(:,3),CBp(:,3)]; % Translation

fill3(RWBp_r(1,:),RWBp_r(2,:),RWBp_r(3,:),(1/255)*[68,68,68]); % Drawing the right wheel
fill3(RWBp_l(1,:),RWBp_l(2,:),RWBp_l(3,:),(1/255)*[68,68,68]); % Drawing the left wheel

% Plot the platform

otbot_circle(xs(1),xs(2),0.3);

% Drawing the orientation line
plot3([xs(1),0.3*cos(xs(3))+xs(1)],[xs(2),0.3*sin(xs(3))+xs(2)],[0,0],'color','r')

% Drawing obstacles
rObst = get_round_obstacles();
%lObst = get_line_obstacles();

%add moving obstacle visualization
for k = 1:size(rObst_t,1) % round fixed obstacles
    otbot_circle_v2(rObst_t(k,i,1), rObst_t(k,i,2), rObst_t(k,i,3), 'r');
end         

for k = 1:size(rObst,1) % round obstacles moving with time
    otbot_circle_v2(rObst(k,1), rObst(k,2), rObst(k,3), 'r');
end

% Drawing goal trajectory
% step = 0:0.01:100;
% goal_trajectory = circle_traj(step, m);
plot(end_pos(1),end_pos(2),'rx', 'MarkerSize', 10, 'linewidth', 1.5); 


% Drawing goal trajectory
step = 0.1:0.01:9.9;

switch traj_var
    case 'fixed'
        end_traj = zeros(3,size(step,1));
        end_traj(1,:) = -6;
        end_traj(2,:) = -6;
        plot(end_traj(1, :),end_traj(2, :),'rx', 'MarkerSize', 10);    
        
    case 'circle'
        end_traj = circle_traj(step, m);
        plot(end_traj(1, :),end_traj(2, :),'LineWidth', 0.75, 'Color', [0 1 0]);    
        
    case 'square' 
        end_traj = zeros(3,length(step));
        for i = 1:length(step)
            end_traj(:,i) = square_traj(step(i));
        end
        plot(end_traj(1, :),end_traj(2, :),'LineWidth', 0.75, 'Color', [0 1 0]);    
    
    case 'M_traj'
        end_traj = zeros(3,length(step));
        for i = 1:length(step)
            [end_traj(:,i), ~] = M_traj(step(i));
        end
        plot(end_traj(1, :),end_traj(2, :),'LineWidth', 0.75, 'Color', [0 1 0]);    
   
    case 'Line_traj'
        end_traj = zeros(3,length(step));
        for i = 1:length(step)
            [end_traj(:,i), ~] = Line_traj(step(i));
        end
        plot(end_traj(1, :),end_traj(2, :),'LineWidth', 0.75, 'Color', [0 1 0]);    
    
    case 'TrajOpt'
        end_traj = soln.interp.state(step);       
        plot(end_traj(1, :),end_traj(2, :),'LineWidth', 0.75, 'Color', [0 1 0]);  

    otherwise
        end_traj = zeros(3,length(step));
        for i = 1:length(step)
            [end_traj(:,i), ~] = goal_traj(step(i));
        end
        plot(end_traj(1, :),end_traj(2, :),'LineWidth', 0.75, 'Color', [0 1 0]);  
   
end


hold off;
end

