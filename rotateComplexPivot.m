clear all ; close all; clc;
z0 = 2+1i;  % point in complex plane
p = -1 +i*0.5; % pivot point


plot2Dline1Complex(z0 , [-5, 5], 'b');  hold on; grid on; axis([-5 5 -5 5]);

for theta_deg = 0:6:360
  theta = deg2rad(theta_deg); % rotation angle
  z=rotComplexAroundPivot(z0, p, theta) % call rotation around pivot function

  clf; hold on; grid on;

  plot2Dline1Complex(z , [-5, 5], 'b');  hold on; grid on; axis([-5 5 -5 5]); %plot the rotated line
  plot((abs(p)*cos(angle(p))), (abs(p)*sin(angle(p))), 'ro', 'markersize', 10, 'markerfacecolor', 'r');  %plot pivot in red

  box on;
  drawnow;
end
