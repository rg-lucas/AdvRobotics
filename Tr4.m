clear all ; close all; clc;

phi = deg2rad(0);
d=1;  % hessian coordinates of a line
L=[cos(phi);sin(phi);-d];% homogeneous coordinetes of a line
nL=normalize_homogeneous_line(L);
p=[1;0];  % point in cartesian coordinates
plot2Dline( nL , [-2,2],'k');hold on ; grid on ;% plot the line in black
plot( p(1),p(2), 'ro', 'markersize',10, 'markerfacecolor','r');%plot the point in red
axis([-2 2 -2 2]);


for theta_deg = 0:2:360
  theta = deg2rad(theta_deg);% rotation angle
  q = rotation_matrix(theta)*p; % rotate the point about the origin

  R = expm ( skew2D(theta));
  t = q;  % translate vector
  H = [ R, -R*t+t; zeros(1,2), 1];
  % transform the line
  L_prime = inv(H')*nL;
  nL_prime = normalize_homogeneous_line( L_prime);

  clf;% clear the figure
  hold on ; grid on;

  %plot the tranformed line and point
  L=[cos(theta+phi);sin(theta+phi);-d];
  nL=normalize_homogeneous_line(L);
  plot2Dline(nL_prime , [-2 , 2],'k');
  plot(q(1),q(2),'ro','markersize',10,'markerfacecolor','r');
  axis([-2 2 -2 2]);box on;
  pause(0.01);% wait for 0.01 seconds
  drawnow; % draw everything now
endfor
