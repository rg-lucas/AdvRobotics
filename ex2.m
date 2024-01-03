% rigid-body animation of a line about a  point which pivot around the origin
clear all ; close all; clc;
pivot=[1;1];  % point in cartesian coordinates

% hessian coordinates of a line
phi = deg2rad(0);
d=1;
% homogeneous coordinetes of a line
L=[cos(phi);sin(phi);-d];
nL=normalize_homogeneous_line(L);
plot2Dline( nL , [-2,2],'k');hold on ; grid on ; % plot the line in black
plot( pivot(1),pivot(2), 'ro', 'markersize',10, 'markerfacecolor','r');%plot the point in red
%axis limits
axis([-3 3 -3 3]);


for theta_deg = 0:1:360
% homogeneous transformation about the pivot point
  theta = deg2rad(theta_deg); % rotation angle
  q=rot2(theta)*pivot;

  R = expm ( skew2D(theta));
  t=q;  % translation vector
  H = [ R, -R*t+t; zeros(1,2), 1];
  % transform the line
  L_prime = inv(H')*nL;
  nL_prime = normalize_homogeneous_line( L_prime);

  clf;  % clear the figure
  hold on ; grid on;

  %plot the tranformed line and point
  L=[cos(theta+phi);sin(theta+phi);-d];
  nL=normalize_homogeneous_line(L);
  plot2Dline(nL_prime , [-2 , 2],'k');
  plot(q(1),q(2),'ro','markersize',10,'markerfacecolor','r');
  axis([-2 2 -2 2]);box on;
  pause(0.01);
  drawnow;
endfor
