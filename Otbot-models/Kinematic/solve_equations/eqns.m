function dy = eqns(t, y, sm)
% equations dq = f(q,v)  -> y = (q;v) 

alpha = y(3);
varphi_p = y(6);

q = [y(1); y(2); y(3); y(4); y(5); y(6)];

% get M for this step
M_IIK = sm.MIIKmatrix(alpha, varphi_p);

% get control variables for this step
u = control(t, y); 
   
dy = zeros(6, 1);
dy(1:3) = u;
dy(4:6) = M_IIK * u;

end