function [dz, dzGrad] = OtbotDynamics(z, u, sm)
% equations dq = f(q,v)  -> q = (q;v) 

% impose varphi_l(0) = 0
% will not work if phi_l is supposed to be at another position initially 
K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1)); 

% get x from z
x = sm.psi(K_0, z(3,:), z(6,:), z(7,:), z(8,:), z(1,:), z(4,:), z(2,:), z(5,:));

xdot = x(7:12, :);

% Get xdotdot
xdotdot = sm.forward_dyn(x(3,:),x(9,:),u(2,:),u(3,:),u(1,:),x(6,:),x(12,:),x(7,:),x(8,:));

dx = [xdot ; xdotdot];

% compute dz from dx 
dz = sm.L *dx;


if nargout == 2   % Analytic gradients    dxGrad = [nState, 1+nx+nu, nTime]
    
    %dzGrad = zeros(size(z,1), 1 + size(z,1) + size(u,1), size(z,2));
    dzGraddt = zeros(size(z,1), 1, size(z,2) );  % is this correct ??
    
    % get x from z
    K_0 = -1 * sm.K(z(3,1), 0, z(7,1), z(8,1)); 
    x = sm.psi(K_0, z(3,:), z(6,:), z(7,:), z(8,:), z(1,:), z(4,:), z(2,:), z(5,:));
    
    dzGraddz = zeros(size(z,1), size(z,1), size(z,2));
    dzGraddu = zeros(size(z,1), size(u,1), size(z,2));
    for k = 1:size(z,2)  % need loop otherwise error on reshape

        dzGraddz(:,:,k) = sm.dgdz(x(3,k), x(9,k), u(2,k),u(3,k),u(1,k), x(6,k), x(12,k), x(7,k),x(8,k));
        dzGraddu(:,:,k) = sm.dgdu(x(3,k), x(6,k));
    end
%         dzGraddz = sm.dgdz(x(3,:), x(9,:), u(2,:),u(3,:),u(1,:), x(6,:), x(12,:), x(7,:),x(8,:));
%     dzGraddu = sm.dgdu(x(3,:), x(6,:));
    
    dzGrad = cat(2, dzGraddt, dzGraddz, dzGraddu);
    
end

end