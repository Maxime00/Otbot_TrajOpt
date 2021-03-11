function roundObst = get_round_obstacles()
%GET_ROUND_OBSTACLES Outputs an array of round obstacles organized as such : 
% col 1 = x position
% col 2 = y position
% col 3 = radius 


numObst = 0;

roundObst = zeros(numObst,3);

if numObst ~= 0
        roundObst(:,1) = [ -9 ; -6;-3;0;3;6;9];  
        roundObst(:,2) = [1; -1;1;-1;1;-1;1];
        roundObst(:,3) = [1;1;1;1;1;1;1];
end

end
% obstacles for backtrapped problem : need initial traj 
%         roundObst(:,1) = [ 3 ; 0;0;2;2;-2;-2];  
%         roundObst(:,2) = [0; 3;-3;2;-2;2;-2];
%         roundObst(:,3) = [1.1;1.1;1.1;1.1;1.1;1.1;1.1];

% obstacles for forced zig zag 
%         roundObst(:,1) = [-7;-3;7;3];  
%         roundObst(:,2) = [-3;-6;3;6];
%         roundObst(:,3) = [3.5;2; 3.5; 2];

% % obstacles for corridor with obstacles 
%         roundObst(:,1) = [ -9 ; -6;-3;0;3;6;9];  
%         roundObst(:,2) = [1; -1;1;-1;1;-1;1];
%         roundObst(:,3) = [1;1;1;1;1;1;1];
