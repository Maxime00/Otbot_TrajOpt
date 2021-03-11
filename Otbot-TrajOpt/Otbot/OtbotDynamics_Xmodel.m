function dq = OtbotDynamics_Xmodel(q, u, sm)
% equations dq = f(q,v)  -> q = (q;v) 

qdot = q(7:12, :);

% Get qdotdot
qdotdot = sm.forward_dyn(q(3,:),q(9,:),u(2,:),u(3,:),u(1,:),q(6,:),q(11,:),q(12,:),q(10,:));

dq = [qdot ; qdotdot];

end