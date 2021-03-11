function video_sim(sln, h, m)

time=cputime;
% Animation
% h is the sampling time 
% n is the scaling factor in order not to plot with the same step
% than during the integration with ode45

fs=30;
n=round(1/(fs*h));

times = sln.T;
states = sln.Y;
K = control_hyper_parameters();
fn = sprintf('../Results/Inertia OG/Kinematic - Simple Controller/sim/Simulation for K_x %1.1f, K_y %1.1f, K_a %1.1f.png', K(1,1), K(2,2), K(3,3));

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
        if elapsed > 200
            disp(elapsed);
            disp('took too long to generate the video')
            break
        else
            draw_otbot(m,q)            
            frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
            writeVideo(writerObj, frame); 
        end

end
close(writerObj); % Saves the movie.

end

