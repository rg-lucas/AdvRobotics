% plot a complex plane line through origin
function plot2Dline0Complex( z, realAxis_interval, color)

real_min = realAxis_interval(1);
real_max = realAxis_interval(2);

im_coord_min = real_min*imag(z)/real(z);
im_coord_max = real_max*imag(z)/real(z);

plot([real_min, real_max], [im_coord_min, im_coord_max], color, 'linewidth', 4);
xlabel('Re','fontsize',20);
ylabel('Im','fontsize',20);

endfunction