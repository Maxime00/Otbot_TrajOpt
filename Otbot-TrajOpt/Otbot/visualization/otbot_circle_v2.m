function h = otbot_circle_v2(x,y,r,fillcolor)
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit,'LineWidth', 0.75, 'Color', [0 0 0]);

srtc = strcmp(fillcolor,'NO');
if srtc == 0
    h = fill(xunit,yunit,fillcolor);
end
end

