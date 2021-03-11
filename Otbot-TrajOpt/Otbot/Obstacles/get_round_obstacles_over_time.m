function roundObst = get_round_obstacles_over_time(t)
%GET_ROUND_OBSTACLES Outputs an array of round obstacles organized as such : 
% col 1 = x position
% col 2 = y position
% col 3 = radius 

% randomize this??

numObst =0; % number of Obstacles -- MUST UPDATE
w = 3; % m/s obstacle speed

roundObst = zeros(numObst,3);

if numObst ~= 0
    for i = 1:length(t)
        curr_t = t(i);
          
        roundObst(:,i,1) = [-5.5;0;5.5;-5.5;0;5.5;-5.5;0;5.5];  
        roundObst(:,i,2) = [6+3*sin(w*curr_t);6+3*sin(w*curr_t);6+3*sin(w*curr_t);3*sin(w*curr_t);3*sin(w*curr_t);3*sin(w*curr_t);-6+3*sin(w*curr_t);-6+3*sin(w*curr_t);-6+3*sin(w*curr_t)];
        roundObst(:,i,3) = [2.2;2.2;2.2;2.2;2.2;2.2;2.2;2.2;2.2];

    end
end
% 
%         roundObst(:,i,1) = [6 * cos(w*curr_t); 6 * cos(w*curr_t + 2*pi/3);6 * cos(w*curr_t + 4*pi/3)];  
%         roundObst(:,i,2) = [6* sin(w*curr_t);6* sin(w*curr_t + 2*pi/3);6* sin(w*curr_t+ 4*pi/3)];
%         roundObst(:,i,3) = [1.25;1.25;1.25];
%

% crowded corrdior moving 3 obstacles, speed = 2
%         roundObst(:,i,1) = [-7;0; 7];  
%         roundObst(:,i,2) = [3* sin(w*curr_t);-3* sin(w*curr_t);3* sin(w*curr_t)];
%         roundObst(:,i,3) = [2;2;2];


% crowded corrdior moving 3 obstacles, speed = 2 or 3
%         roundObst(:,i,1) = [-8.5;-4;0;4;8.5];  
%         roundObst(:,i,2) = [3* sin(w*curr_t);-3* sin(w*curr_t);3* sin(w*curr_t);-3* sin(w*curr_t);3* sin(w*curr_t)];
%         roundObst(:,i,3) = [1.5;1.5;1.5;1.5;1.5];

% crowded corrdior moving 7 obstacles - speed = 3
%         roundObst(:,i,1) = [-9; -6;-3;0; 3;6;9];  
%         roundObst(:,i,2) = [3* sin(w*curr_t);-3* sin(w*curr_t);3* sin(w*curr_t);-3* sin(w*curr_t);3* sin(w*curr_t);-3* sin(w*curr_t);3* sin(w*curr_t)];
%         roundObst(:,i,3) = [1;1;1;1;1;1;1];

% insane box 
%         roundObst(:,i,1) = [-5.5;0;5.5;-5.5;0;5.5;-5.5;0;5.5];  
%         roundObst(:,i,2) = [6+3*sin(w*curr_t);6+3*sin(w*curr_t);6+3*sin(w*curr_t);3*sin(w*curr_t);3*sin(w*curr_t);3*sin(w*curr_t);-6+3*sin(w*curr_t);-6+3*sin(w*curr_t);-6+3*sin(w*curr_t)];
%         roundObst(:,i,3) = [2.2;2.2;2.2;2.2;2.2;2.2;2.2;2.2;2.2];

end

