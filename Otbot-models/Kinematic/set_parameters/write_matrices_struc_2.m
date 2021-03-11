load("jofq_matrix")
load("fik_iikofq_matrices")
load("m_struc")
%% Creating symbolic variables

syms l_1 l_2 r alpha varphi_p

%% Seting up

% Now we will be seting up this matrices in a apropiate way in order to use
% them for the simulations.

% First J matrix
Jmatrix = subs(J_of_q,[l_1,l_2,r],[m.l_1,m.l_2,m.r]);

% Save matrices in a structure array
sm.Jmatrix = matlabFunction(Jmatrix);

% Now MFIK matrix
MFIKmatrix = subs(MFIK_of_q,[l_1,l_2,r],[m.l_1,m.l_2,m.r]);

% Save matrices i na structure array
sm.MFIKmatrix = matlabFunction(MFIKmatrix);

% Finally MIIK matrix
MIIKmatrix = subs(MIIK_of_q,[l_1,l_2,r],[m.l_1,m.l_2,m.r]);

% Save matrices i na structure array
sm.MIIKmatrix = matlabFunction(MIIKmatrix);

%% Saveing only the structure object and deleting everything else
save('C:\Users\pereg\Documents\UPC\Master\TFMPractiques\MatlabWD\Sim_Otbot\Kin_Sim\sm_struc.mat',"sm","sm")
clearvars
close all
clc
