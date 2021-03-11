function h = otbot_circle(x,y,r)
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit,'LineWidth', 0.75, 'Color', [0 0 0]);
end

