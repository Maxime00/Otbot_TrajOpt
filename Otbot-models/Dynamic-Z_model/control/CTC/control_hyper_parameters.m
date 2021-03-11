% You can set any hyper parameters of the control function here; you may or
% may not want to use the step_number as the input of the function. 
function [Kp, Kv, Ki] = control_hyper_parameters()
% Set control variables 

Kp = zeros(3,3);

Kp(1,1) = 1.32;%244.9490 ;%2.1;%1.32;%0.4;%
Kp(2,2) = 1.32;%244.9490 ;%1.32;%244.9490 ;%0.4;%
Kp(3,3) = 1.32;%244.9490 ;%1.32;%244.9490 ;%0.4;%

Kv = zeros(3,3);

Kv(1,1) = 2.3;%22.1562;%2.9;%1.2;%2.3
Kv(2,2) = 2.3;%22.1562;%2.3;%22.1562;%1.2;%
Kv(3,3) = 2.3;%22.1562;%2.3;%22.1562;%1.2;%

Ki = zeros(3,3);
% 
% Ki(1,1) = 1.5;%1200;%0.4;%
% Ki(2,2) = 1.5;%1200;%0.4;%1200;%
% Ki(3,3) = 1.5;%1200;%0.4;%1200;%

% LQR solution
% Actc = [zeros(3,3),eye(3,3); zeros(3,3), zeros(3,3)];
% Bctc = [zeros(3,3);eye(3,3)];
% Qctc = [3e5*eye(3,3),zeros(3,3);
%         zeros(3,3), 5*eye(3,3)];
% Rctc = 5*eye(3,3);
% K = lqr(Actc,Bctc,Qctc,Rctc);
% 
% Kp = K(1:3, 1:3);
% Kv = K(1:3, 4:6);
%    


end
