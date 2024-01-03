clear; close all; clc;
% a point in space in homogeneous coorinates
x = [0;0;1;1];
y = [0;0;2;1];

% define Line 1
q = [0;0;0]; % a point on line
u = [1;0;0]; % direction of line
u = u/norm(u); % make it unit

% Euclidean normalized plucker line coordinates
L1 = qU2pluckerLine( q, u);

% unit twist from line
h = 0;
xi = plucker2twist (L1, h);

% homogeneous form of the twist
xi_hat = twist2homogenousform( xi );

% screw transformation
y_primes = [];
theta = linspace (0,pi/2,30);

% define Line 2:
v = [0;1;0;1]; % will be the original direction of line 2

figure;
for i=1:length(theta)
    % rotation of point x around Line 1:
    H1= expm(-theta(i)*xi_hat);
    x_prime = H1*x;

    % rotation of Line 2:
    v_prime = H1*v;
    v_prime = v_prime/norm(v_prime);
    L2 = qU2pluckerLine( x_prime(1:3), v_prime(1:3));

    % rotation of point y around Line 2:
    yi = plucker2twist (L2, h);
    yi_hat = twist2homogenousform( yi );
    H2= expm(2*theta(i)*cos(theta(i)/2)*yi_hat); % Adjustment of theta for rotation over two lines
    y_prime = H2*H1*y;
    y_primes =  [ y_primes, y_prime];

% plot the scene
clf;
% plot axis lines:
plotPluckerLine( L1, 5, [.64,.1,1]); hold on; grid on; axis equal;
plotPluckerLine( L2, 5, [.64,.1,1]); hold on; grid on; axis equal;
% plot points:
plot3(q(1), q(2), q(3), 'ko','markersize',8,'markerfacecolor','b');
plot3(x_prime(1), x_prime(2), x_prime(3), 'ko','markersize',8,'markerfacecolor','b');
plot3(y_prime(1), y_prime(2), y_prime(3), 'ko','markersize',8,'markerfacecolor','r');
% plot 'arm' lines
plot3([q(1), x_prime(1)], [q(2), x_prime(2)], [q(3),x_prime(3)],'k','linewidth',4);
plot3([x_prime(1), y_prime(1)], [x_prime(2), y_prime(2)], [x_prime(3), y_prime(3)],'k','linewidth',4);
%Plot trajectory:
plot3(y_primes(1,:), y_primes(2,:), y_primes(3,:),'r','linewidth',4);
axis([-1 2 -1 2 -1 2 ]); drawnow; %pause(0.00005);
end

