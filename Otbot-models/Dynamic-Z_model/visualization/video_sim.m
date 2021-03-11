function video_sim(sln, h, m,sm, cd, fn, soln, traj_var)

time=cputime;
% Animation
% h is the sampling time 
% n is the scaling factor in order not to plot with the same step
% than during the integration with ode45

fs=30;
n=round(1/(fs*h));

times = sln.T.';
y = sln.Y;

K_0 = -1 * sm.K(y(3,1), 0, y(7,1), y(8,1));
x = sm.psi(K_0, y(3,:), y(6,:), y(7,:), y(8,:), y(1,:), y(4,:), y(2,:), y(5,:));
states = x.';

% get moving obstacle coords
rObst_t = get_round_obstacles_over_time(times);

% set up end goal 
end_pos = states(end-1,:); % last state is NaN

[Kp, Kv, Ki] = control_hyper_parameters();
fn = sprintf('%s/sim/Simulation for %s Kp_x %1.1f, Kp_y %1.1f, Kp_a %1.1f, Kv_x %1.1f, Kv_y %1.1f, Kv_a %1.1f, Ki_x %1.1f, Ki_y %1.1f and Ki_a %1.1f', cd, fn, Kp(1,1), Kp(2,2), Kp(3,3), Kv(1,1), Kv(2,2),Kv(3,3), Ki(1,1), Ki(2,2), Ki(3,3));

% Set up the movie.
writerObj = VideoWriter(fn,'MPEG-4'); % Name it.
%writerObj.FileFormat = 'mp4';
writerObj.FrameRate = fs; % How many frames per second.
open(writerObj);

for i = 1:n:length(times)
        q.x = states(i,1);
        q.y = states(i,2);
        q.alpha = states(i,3);
        q.varphi_r = states(i,4);
        q.varphi_l = states(i,5);
        q.varphi_p = states(i,6);

        elapsed = cputime-time;
        if elapsed > 400
            disp(elapsed);
            disp('took too long to generate the video')
            break
        else
            draw_otbot(m,q, soln, traj_var, end_pos, rObst_t)            
            frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
            writeVideo(writerObj, frame); 
        end

end
close(writerObj); % Saves the movie.

end

