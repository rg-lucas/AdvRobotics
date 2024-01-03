clear all ; close all; clc;
z0 = 2+1i;  % point in complex plane
theta0 = rad2deg(arg(z0)); % initial position

plot2Dline1Complex(z0 , [-5, 5], 'b');  hold on; grid on; axis([-5 5 -5 5]);

for theta_deg = (theta0):5:(theta0+360)
  theta = deg2rad(theta_deg); % rotation angle
  z= abs(z0) * exp( j * (theta)); % rotates z0 1 degree at a time

  clf; hold on; grid on;

  %plot the rotated line
  plot2Dline1Complex(z , [-5, 5], 'b');  hold on; grid on; axis([-5 5 -5 5]);

  box on;
  drawnow;
  % pause(0.01); % 5 seconds for 360 degrees

end
