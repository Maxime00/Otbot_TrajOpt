function [GBp,GPp] = c_o_m_bodies(m,q)
%C_O_M_BODIES Compute the position of centers of mass
%   This function is designed to calculate the centers of mass of the
%   chassis body and the platform body

xs(1)=q.x;
xs(2)=q.y;
xs(3)=q.alpha;
xs(4)=q.varphi_r;
xs(5)=q.varphi_l;
xs(6)=q.varphi_p;

% Defining rotation matrix
Rmat = m.Rzmat(xs(3)-xs(6));

Rmat2 = m.Rzmat(xs(3));

trvec = [xs(1);xs(2);0];

% Center of mass of the Chassis body
GBp = Rmat*[m.x_G; m.y_G; 0] + trvec;

% Center of mass of the platform
GPp = Rmat2*[m.x_F; m.y_F; 0] + trvec;

end

