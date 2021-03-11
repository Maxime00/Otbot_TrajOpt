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
Delta = [MFIKmatrix ; eye(3)];

Mbar = Delta.' * Mmatrix * Lambda;
sm.Mbar = matlabFunction(Mbar);

% new J matrix (used for kin error in traj opt) 
newJmatrix = [eye(3) , -MFIKmatrix];
sm.newJmatrix = matlabFunction(newJmatrix);

% K term used in holonomic constraint : K_0
K = qvec(3) - qvec(6) + (m.r/(2*m.l_2))*(qvec(5)-qvec(4));
sm.K = matlabFunction(K);

% z = psi(x) used in traj opt
syms K_0 % can't be given an expression here since it requires t

psi = [qvec(1:4) ; qvec(4) + ((2*m.l_2)/m.r)*(qvec(6)-qvec(3)-K_0); qvec(6);...
        qdotvec(1:3); MIIKmatrix * qdotvec(1:3)];
sm.psi = matlabFunction(psi);

% x = L*z used in traj opt
L = [eye(3), zeros(3,9) ; zeros(3,6), eye(3), zeros(3,3) ;zeros(2,12)];
L(7,6) = 1;
L(8,4) = 1;
sm.L = L;

% Find Lambda dot
syms f1(t) f2(t)
Lambda_of_t = subs(Lambda,[alpha varphi_p],[f1(t) f2(t)]);
dLambdadt = diff(Lambda_of_t,t);
% Substitute d/dt of f1 and f2 by the original variables using dot notation
Lambdadot = subs(dLambdadt,[f1(t) f2(t) diff(f1,t) diff(f2,t)], [alpha varphi_p alpha_dot varphi_dot_p]);

Cbar = Delta.' * ( Mmatrix* Lambdadot +  Cmatrix * Lambda);
sm.Cbar = matlabFunction(Cbar);

% Find MIIKdot
MIIK_of_t = subs(MIIKmatrix, [alpha varphi_p],[f1(t) f2(t)]);
dMIIKdt = diff(MIIK_of_t, t);
MIIKdot = subs(dMIIKdt,[f1(t) f2(t) diff(f1,t) diff(f2,t)], [alpha varphi_p alpha_dot varphi_dot_p]);
sm.MIIKdot = matlabFunction(MIIKdot);

%% Analytical gradients  df(x,u)/dx    df(x,u)/du

M_big = [Mbar,zeros(3,3); -MIIKmatrix, eye(3)];
F = [uvec - Cbar*qdotvec(1:3); MIIKdot*qdotvec(1:3)];

% dq_dotdot/dq_i
dqdotdot_dq = sym(zeros(6,6)); 
for i =1:nq
    dqdotdot_dq(:,i) = inv(M_big) *(-diff(M_big, qvec(i)) * (inv(M_big)*F) + diff(F,qvec(i)));
end

% dq_dotdot/dqdot_i
dqdotdot_dqdot = sym(zeros(6,6)); 
for i =1:nq
    dqdotdot_dqdot(:,i) = inv(M_big) *diff(F,qdotvec(i));
end

% dq_dotdot/du
dqdotdot_du = sym(zeros(6,3)); 
for i =1:3
    dqdotdot_du(:,i) = inv(M_big) *diff(F,uvec(i));
end

dfdx = [ zeros(6,6), eye(6) ; dqdotdot_dq , dqdotdot_dqdot];
%sm.dfdx = matlabFunction(dfdx);
dfdu = [zeros(6,3) ; dqdotdot_du];
%sm.dfdu = matlabFunction(dfdu);


%% Analitycal gradients in Z state dg(z,u)/dx    dg(z,u)/du

% dpsi(z)/dz
dpsidz = jacobian(psi, [x, y, alpha, x_dot, y_dot, alpha_dot, varphi_p, varphi_r]);

dgdz = L * dfdx * dpsidz;
dgdz = simplify(dgdz);
sm.dgdz = matlabFunction(dgdz);

dgdu = L*dfdu;
dgdu = simplify(dgdu);
sm.dgdu = matlabFunction(dgdu);


%% Add inverse and forward functions here

% Full model functions 
forward_func = inv([Mbar,zeros(3,3); -MIIKmatrix, eye(3)])*[uvec - Cbar*qdotvec(1:3); MIIKdot*qdotvec(1:3)];
forward_func  = simplify(forward_func);
sm.forward_dyn = matlabFunction(forward_func);


%% Add the experssion of the Kinetic Energy
Texpr = subs(T,[I_b, I_p, I_a, I_t, l_1, l_2, m_b, m_w, m_p, x_G, y_G, x_F, y_F, r], [m.I_b, m.I_p, m.I_a, m.I_t, m.l_1, m.l_2, m.m_b, m.m_w, m.m_p, m.x_G, m.y_G, m.x_F, m.y_F, m.r]);
% Save kinetic energy expression in structure array
sm.Texpr = matlabFunction(Texpr);

%% Saveing only the structure object and deleting everything else
save('model\sm_struc.mat',"sm","sm")
clearvars
close all
clc
