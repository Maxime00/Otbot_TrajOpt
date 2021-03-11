% LQR solution
% COmpute K values from lqr function 
% clearvars 
% close all
% clc

Actc = [zeros(3,3),eye(3,3); zeros(3,3), zeros(3,3)];
Bctc = [zeros(3,3);eye(3,3)];
Qctc = [3e5*eye(3,3),zeros(3,3);
        zeros(3,3), 5*eye(3,3)];
Rctc = 5*eye(3,3);
K = lqr(Actc,Bctc,Qctc,Rctc);

Kp = K(1:3, 1:3);
Kv = K(1:3, 4:6);

%% Task space 

Actc = [zeros(3,3),eye(3,3); zeros(3,3), zeros(3,3)];
Bctc = [zeros(3,3);eye(3,3)];

EIG_set = [-1.1,-1.1,-1.1, -1.2, -1.2, -1.2]; % Desired eigenvalues

K = place(Actc,Bctc,EIG_set)

%% Full model
Actc = [zeros(3,3),eye(3,3), zeros(3,3); zeros(3,3), zeros(3,3), eye(3,3); zeros(3,3), zeros(3,3), zeros(3,3)];
Bctc = [zeros(3,3) ; zeros(3,3) ; eye(3,3)];

EIG_set = [-1,-1.1,-1.2, -1.3, -1.4, -1.5, -1.6, -1.7, -1.8]; % Desired eigenvalues

K = place(Actc,Bctc,EIG_set)

Kp = K(1:3, 1:3);
Kv = K(1:3,4:6);
Ki = K(1:3, 7:9);