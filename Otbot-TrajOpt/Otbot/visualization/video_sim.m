function video_sim(times,states, fn, h, XYbound, initial_guess)

time=cputime;
% Animation
% h is the sampling time 
% n is the scaling factor in order not to plot with the same step
% than during the integration with ode45

load("m_struc")

% set up end goal 
end_pos = states(end,:);

% get moving obstacle coords
rObst_t = get_round_obstacles_over_time(times);

fs=30; % Number of frames per second 
n=round(1/(fs*h));

% get COM for plotting
% Set, ploting factor:
Gpf = 8; % Plot the com every Gpf frame i.e every 2 frames (Gpf = 2...)
n2 = Gpf*n;
i_com = 1;
com = zeros(3,round(length(times)/n2)+2); % store all com to plot them all

ffn = sprintf('Otbot/Results/Otbot/videos/Sim for %s', fn );

% Set up the movie.
writerObj = VideoWriter(ffn,'MPEG-4'); % Name it.
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
            % add com plot
            draw_otbot(m,q, end_pos, i, rObst_t, XYbound, initial_guess)         
            multi_circles(com(:,1:i_com),m.l_2/6,'g')
            if mod(i-1,n2) == 0 % adds COM position on video
                [~,GPpi] = c_o_m_bodies(m,q);
                com(:,i_com) = GPpi;
                i_com = i_com +1;
            end 
            if (i == 1) || ((i >= length(times)/2-n)&& i <= (length(times)/2+n)) || (i >= length(times)-n)
                i_str = sprintf('%.0f_', i);
                saveas(gcf, strcat('Otbot/Results/Otbot/videos/Frame-',i_str,fn,'.eps'), 'epsc')
                saveas(gcf, strcat('Otbot/Results/Otbot/videos/Frame-',i_str,fn,'.png'))
            end
            frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
            writeVideo(writerObj, frame); 
        end

end
close(writerObj); % Saves the movie.

end

