clear; close all;
p=[1;0];
theta=linspace(0, 2*pi, 180);
figure;

% hessian coordinates of a line
phi= deg2rad (90); d = 1;
% homogeneous coordinates of a line
L = [cos(phi); sin(phi);-d];
nL= normalize_homogeneous_line ( L );

for i=1:length(theta)

  Pn = rotation_matrix(theta(i))*p;
  clf;
  theta1 = deg2rad(89.999);
  R = expm( skew2D(theta1) );
  t = [p(1)+Pn(1); p(2)+Pn(2)]; % translation vector
  H = [R, t; zeros(1,2), 1 ];

  % transform the line
  L_prime =inv(H')*nL;
  nL_prime = normalize_homogeneous_line (L_prime ),

  %plot the line in black
  plot2Dline(nL_prime, [-2,2], 'k' ); hold on;

  %plot the point in red
  plot ( Pn(1), Pn(2), 'ro', 'markersize',8,'markerfacecolor','r');
  axis ([-2 2 -2 2]); grid on;  hold on;

  pause(0.01);
  drawnow;

endfor
