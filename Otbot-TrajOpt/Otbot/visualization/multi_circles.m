function [] = multi_circles(XYZmatrix,radius,fillcolor)
%MULTI_CIRCLES Function to plot multiple circles at desired positions
%   This function is designed to plot a circle in each specified position
%   with the same radius and the same color. Warning: Z input will be
%   dismissed for now

Dim = size(XYZmatrix);


hold on
for iaux1 = 1:Dim(2)
    otbot_circle_v2(XYZmatrix(1,iaux1),XYZmatrix(2,iaux1),radius,fillcolor);
end
hold off

end

