% plot a complex plane line not through origin
function d = plot2Dline1Complex( z0, realAxis_interval, color)

theta = arg(z0);
cos_theta = cos(theta);
sin_theta = sin(theta);
d = norm(z0);

nL = [ cos_theta; sin_theta; -d];
plot2Dline(nL, realAxis_interval, color);
xlabel('Re','fontsize',40);
ylabel('Im','fontsize',40);

endfunction