load("jofq_matrix")
load("fik_iikofq_matrices")
load("m_struc")
load("dynamic_model_workspace")
%% Creating the missing symbolic variables

syms r

%% Setting up

% Now we will be seting up this matrices in a apropiate way in order to use
% them for the simulations.

% First J matrix
Jmatrix = subs(J_of_q,[l_1,l_2,r],[m.l_1,m.l_2,m.r]);

% Save matrices in a structure array
sm.Jmatrix = matlabFunction(Jmatrix);

% Now MFIK matrix
MFIKmatrix = subs(MFIK_of_q,[l_1,l_2,r],[m.l_1,m.l_2,m.r]);

% Save matrices in a structure array
sm.MFIKmatrix = matlabFunction(MFIKmatrix);

% Finally MIIK matrix
MIIKmatrix = subs(MIIK_of_q,[l_1,l_2,r],[m.l_1,m.l_2,m.r]);

% Save matrices in a structure array
sm.MIIKmatrix = matlabFunction(MIIKmatrix);

% Now Jdot matrix
Jdotmatrix = subs(Jdot,[l_1,l_2,r],[m.l_1,m.l_2,m.r]);

% Save matrices in a structure array
sm.Jdotmatrix = matlabFunction(Jdotmatrix);

% Now Mass matrix
Mmatrix = subs(M,[I_b, I_p, I_a, I_t, l_1, l_2, m_b, m_w, m_p, x_G, y_G, x_F, y_F, r], [m.I_b, m.I_p, m.I_a, m.I_t, m.l_1, m.l_2, m.m_b, m.m_w, m.m_p, m.x_G, m.y_G, m.x_F, m.y_F, m.r]);

% Save matrices in a structure array
sm.Mmatrix = matlabFunction(Mmatrix);

%Now we include E matrix also
sm.Ematrix = Ematrix;

% Now Cmat
Cmatrix = subs(Cmat,[I_b, I_p, I_a, I_t, l_1, l_2, m_b, m_w, m_p, x_G, y_G, x_F, y_F, r], [m.I_b, m.I_p, m.I_a, m.I_t, m.l_1, m.l_2, m.m_b, m.m_w, m.m_p, m.x_G, m.y_G, m.x_F, m.y_F, m.r]);

% Save matrices in a structure array
sm.Cmatrix = matlabFunction(Cmatrix);

% Add matrices for CTC law

Lambda = [eye(3) ; MIIKmatrix];

Mbar = Lambda.' * Mmatrix * Lambda;
sm.Mbar = matlabFunction(Mbar);

% Find Lambda dot
syms f1(t) f2(t)
Lambda_of_t = subs(Lambda,[alpha varphi_p],[f1(t) f2(t)]);
dLambdadt = diff(Lambda_of_t,t);
% Substitute d/dt of f1 and f2 by the original variables using dot notation
Lambdadot = subs(dLambdadt,[f1(t) f2(t) diff(f1,t) diff(f2,t)], [alpha varphi_p alpha_dot varphi_dot_p]);

Cbar = Lambda.' * Mmatrix* Lambdadot +  Lambda.' * Cmatrix * Lambda;
sm.Cbar = matlabFunction(Cbar);

% Find MIIKdot

MIIK_of_t = subs(MIIKmatrix, [alpha varphi_p],[f1(t) f2(t)]);
dMIIKdt = diff(MIIK_of_t, t);
MIIKdot = subs(dMIIKdt,[f1(t) f2(t) diff(f1,t) diff(f2,t)], [alpha varphi_p alpha_dot varphi_dot_p]);
sm.MIIKdot = matlabFunction(MIIKdot);

%% Add inverse and foward functions here

% Full model functions 
forward_func = [eye(6),zeros(6,3)]*inv([Mmatrix,Jmatrix.'; Jmatrix, zeros(3,3)])*[Ematrix*uvec - Cmatrix*qdotvec; -Jdotmatrix*qdotvec];
forward_func  = simplify(forward_func);
sm.forward_dyn = matlabFunction(forward_func);

syms qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 
qdd = [qdd1 ;qdd2; qdd3;qdd4; qdd5; qdd6];
inv_func = [eye(3) , zeros(3,3)] * inv([Ematrix , -Jmatrix.']) * (Mmatrix *qdd  + Cmatrix*qdotvec);
inv_func = simplify(inv_func);
sm.inv_dyn = matlabFunction(inv_func);

% Task Space model functions

pdd = Mbar\( MIIKmatrix.' * uvec - Cbar * qdotvec(1:3));
phidd = MIIKdot * qdotvec(1:3) + MIIKmatrix * qvec(1:3);
forward_tsm = [pdd ; phidd];
sm.forward_dyn_tsm = matlabFunction(forward_tsm);

inv_tsm = MFIKmatrix.' * (Mbar * qdd(1:3) + Cbar * qdotvec(1:3));
sm.inv_dyn_tsm = matlabFunction(inv_tsm);

%% Add the experssion of the Kinetic Energy
Texpr = subs(T,[I_b, I_p, I_a, I_t, l_1, l_2, m_b, m_w, m_p, x_G, y_G, x_F, y_F, r], [m.I_b, m.I_p, m.I_a, m.I_t, m.l_1, m.l_2, m.m_b, m.m_w, m.m_p, m.x_G, m.y_G, m.x_F, m.y_F, m.r]);
% Save kinetic energy expression in structure array
sm.Texpr = matlabFunction(Texpr);

%% Saveing only the structure object and deleting everything else
save('model\sm_struc.mat',"sm","sm")
clearvars
close all
clc
