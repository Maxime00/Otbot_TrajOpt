function [  ] = draw_otbot( m, q, end_pos, i, rObst_t, XYbound, initial_guess)
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
limX = XYbound(1)+1;
limY = XYbound(2)+1;
axis([-limX limX -limY limY]);

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

% Drawing goal trajectory
% step = 0:0.01:100;
% goal_trajectory = circle_traj(step, m);
plot(end_pos(1),end_pos(2),'rx', 'MarkerSize', 10, 'linewidth', 1.5);    

% Drawing XY bounds
plot3([-XYbound(1), XYbound(1)],[XYbound(2), XYbound(2)], [0,0], 'color','r', 'LineWidth', 2);
plot3([-XYbound(1), XYbound(1)],[-XYbound(2), -XYbound(2)], [0,0], 'color','r', 'LineWidth', 2);
plot3([XYbound(1), XYbound(1)],[-XYbound(2), XYbound(2)], [0,0], 'color','r', 'LineWidth', 2);
plot3([-XYbound(1), -XYbound(1)],[-XYbound(2), XYbound(2)], [0,0], 'color','r', 'LineWidth', 2);

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

% plot inital guess
for k = 2:(size(initial_guess,2)-1)
    plot(initial_guess(1,k),initial_guess(2,k),'gx', 'MarkerSize', 8, 'linewidth', 1.5);    
end

% for k = 1:size(lObst,1) % obsolete
%         plot3([lObst(k,3), lObst(k,1)*lObst(k,3) + lObst(k,2)],[lObst(k,4), lObst(k,1)*lObst(k,4) + lObst(k,2)], [0,0], 'color','r', 'LineWidth', 10);
% end


hold off;
end

