clear all ; close all; clc;
P = [1; 0];  % point in cartesian coordinates

% hessian coordinates of a line
d = 1;
phi = deg2rad(0);

% homogeneous coordinetes of a line
L = [cos(phi); sin(phi); -d];
nL = normalize_homogeneous_line(L);

plot2Dline(nL , [-2, 2], 'k');  % plot the line in black
hold on;
grid on;
plot(P(1), P(2), 'ro', 'markersize', 10, 'markerfacecolor', 'r');  %plot the point in red

%axis limits
axis([-2 2 -2 2]);


for theta_deg = 0:2:360
% homogeneous transformation about the P point
  theta = deg2rad(theta_deg); % rotation angle
  q = rot2(theta) * P;

  R = expm ( skew2D(theta));
  t = q;  % translation vector
  H = [ R, -R*t+t; zeros(1,2), 1];
  % transform the line
  L_prime = inv(H') * nL;
  nL_prime = normalize_homogeneous_line(L_prime);

  clf;  % clear the figure
  hold on;
  grid on;

  %plot the tranformed line and point
  L = [cos(theta+phi); sin(theta+phi); -d];
  nL = normalize_homogeneous_line(L);
  plot2Dline(nL_prime, [-2, 2], 'k');
  plot(q(1), q(2), 'ro', 'markersize', 10, 'markerfacecolor', 'r');
  axis([-2 2 -2 2]);


  box on;
  drawnow;

end
