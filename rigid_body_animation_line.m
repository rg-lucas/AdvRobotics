% rigid-body animation of a line around a pivot point
clear; close all; clc;
pivot = [1;1];
phi = deg2rad(0.1); d=1.5; %hessian coordinates of a line
L = [cos(phi); sin(phi); -d]; % homogeneous coordinates of a line
nL = normalize_homogeneous_line(L);

plot2Dline(nL, [-2,2], 'k'); hold on; grid on; % plot the line in black
plot(pivot(1), pivot(2), 'ro', 'markersize', 10, 'markerfacecolor', 'r');
axis([-2 2 -2 2]);

for theta_deg = 0:2:360
   % homogeneous transformation around the pivot point
   theta = deg2rad(theta_deg);
   R = expm(skew2D(theta));
   t = pivot; % translation vector
   H = [R, -R*t+t; zeros(1,2),1];

   %transform the line
   L_prime = inv(H')*nL;
   nL_prime = normalize_homogeneous_line( L_prime);

   % plot the transformed line in red
   clf; hold on; grid on;
   plot2Dline(nL_prime, [-1,2], 'k');
   plot(pivot(1), pivot(2), 'ro', 'markersize', 10, 'markerfacecolor', 'r');
   axis([-2 2 -2 2]); box on;
   pause(0.05);drawnow;
   endfor

