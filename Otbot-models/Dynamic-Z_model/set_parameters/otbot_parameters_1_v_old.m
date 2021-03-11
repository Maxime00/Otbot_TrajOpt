%% Using function to_file to create the file with otbot physical parameters
m.I_b = 0.018;   % Central moment of inertia of the chassis body about axis 3 [kg*m^2]
m.I_p = 0.03; % Central moment of inertia of the platform body about axis 3" [kg*m^2]
m.I_a = 0.00035; % Axial moment of inertia of one wheel [kg*m^2]
m.I_t = 0.000625; % Twisting moment of inertia of one wheel [kg*m^2]

m.l_1 = 0.2; % Pivot offset relative to the wheels axis [m]
m.l_2 = 0.1; % One half of the wheels separation [m]

m.m_b = 2.5; % Mass of the chassis base [kg]
m.m_w = 0.5; % Mass of one wheel [kg]
m.m_p = 1.5; % Mass of the platform [kg]

m.x_G = 0; % x coord of the c.o.m. of the chassis body in the chassis frame [m]
m.y_G = 0; % y coord of the c.o.m. of the chassis body in the chassis frame [m]

m.x_F = 0; % x coord of the c.o.m. of the platform body in the platform frame [m]
m.y_F = 0; % y coord of the c.o.m. of the platform body in the platform frame [m]

m.r = 0.05;   % Wheel radius [m]

% Parameters to Draw

% Matrix to define the body in relative frame with the pivot point at the
% origin
m.CBprel = [0, -m.l_1, -m.l_1;
          0, -m.l_2, +m.l_2;
          zeros(1,3)];
      
% Matrix to define the body in relative frame with the pivot point at the
% origin
       
m.RWBprel = [ + m.r,        + m.r,        - m.r,        - m.r ;
           + (m.l_2)/6, - (m.l_2)/6,  - (m.l_2)/6, + (m.l_2)/6;
           zeros(1,4)];
       
% Generic rotation matrix Rz

syms ang

Rzmat = [cos(ang), -sin(ang), 0;
         sin(ang), cos(ang),  0;
          0,          0,      1];
      
m.Rzmat = matlabFunction(Rzmat);


%% Saving m structure
save('model\m_struc.mat',"m","m")
clearvars
close all
clc

%% Run the other script in order to update matrices

write_matrices_struc_2
