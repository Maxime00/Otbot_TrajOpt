% You can set any hyper parameters of the control function here; you may or
% may not want to use the step_number as the input of the function. 
function K = control_hyper_parameters()
% Set control variables and setpoint here ?
K = eye(3);

K(1,1) = 4;
K(2,2) = 4;
K(3,3) = 4;


end
