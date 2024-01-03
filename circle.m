function drawCircle = circle(x,y,r)
close all;

hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit, 'linewidth', 2, 'k');
axis([0 20 0 20]);

daspect([1,1,1])

hold off

endfunction
