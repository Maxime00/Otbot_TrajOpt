function [obj, objGrad] = OtbotPathObj(t,u, obj_func)
% [obj, objGrad] = pathObjective(u)
%
% Computes the objective function (and gradients) for the simple pendulum
%

forceterm_coeff = 0.0001;

switch obj_func
    case "time-forceterm_"
        obj = ones(size(t))+ forceterm_coeff.*(u(1,:).^2 +u(2,:).^2 + u(3,:).^2);% % +  0.00001.*(u(1,:).^2 +u(2,:).^2 + u(3,:).^2)); % 
    case "time-forceterm-phi-p_"
        obj = ones(size(t))+ forceterm_coeff.*( u(3,:).^2);
    case "force-squared_"
        obj = u(1,:).^2 +u(2,:).^2 + u(3,:).^2;
    case "time_"
        obj = ones(size(t));
    case "force-derivative_"
        u_dot = zeros(3,size(t,2));
        for i = 1:(size(t,2)-1)
            u_dot(:,i+1) = (u(:,i+1) - u(:,i))/t(2);
        end
        obj = u_dot(1,:).^2 +u_dot(2,:).^2 + u_dot(3,:).^2;
end


if nargout == 2  % Analytic gradients
    
    switch obj_func
        case "time-forceterm_"
            objGraddt = zeros(1, size(t,2)); 
            objGraddz = zeros(8, size(t,2)); 
            objGraddu = forceterm_coeff.*[ 2*u(1,:) ; 2*u(2,:) ; 2*u(3,:) ]; 
            objGrad = cat(1, objGraddt, objGraddz, objGraddu);
        
       case "time-forceterm-phi-p_"
            objGraddt = zeros(1, size(t,2)); 
            objGraddz = zeros(8, size(t,2)); 
            objGraddu = forceterm_coeff.*[ zeros(1, size(t,2)) ; zeros(1, size(t,2)) ; 2*u(3,:) ]; 
            objGrad = cat(1, objGraddt, objGraddz, objGraddu);
    
        case "force-squared_"
            objGraddt = zeros(1, size(t,2)); 
            objGraddz = zeros(8, size(t,2)); 
            objGraddu = [ 2*u(1,:) ; 2*u(2,:) ; 2*u(3,:) ]; 
            objGrad = cat(1, objGraddt, objGraddz, objGraddu);

        case "time_"
            objGraddt = zeros(1, size(t,2)); 
            objGraddz = zeros(8, size(t,2)); 
            objGraddu = zeros(size(u,1), size(t,2)); 
            objGrad = cat(1, objGraddt, objGraddz, objGraddu);
            
        case "force-derivative_"
            objGraddt = zeros(1, size(t,2)); 
            objGraddz = zeros(8, size(t,2)); 
            u_dot = zeros(3,size(t,2));
            for i = 1:(size(t,2)-1)
                u_dot(:,i+1) = (u(:,i+1) - u(:,i))/t(2);
            end
            objGraddu = [ 2*u_dot(1,:) ; 2*u_dot(2,:) ; 2*u_dot(3,:) ]; 
            objGrad = cat(1, objGraddt, objGraddz, objGraddu);
    end
    
end

end