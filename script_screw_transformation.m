clear; close all; clc;
% a point in space in homogeneous coorinates
x = [5 - rand(3,1); 1 ];

%line parameters
q = rand(3,1); % a point on line
u = rand(3, 1); % direction of line
u = u/norm(u); % make it unit

% Euclidean normalized plucker line coordinates
L = qU2pluckerLine( q, u);

% unit twist from line
h = 1.5;
xi = plucker2twist (L, h);

% homogeneous form of the twist
xi_hat = twist2homogenousform( xi );

% screw transformation
x_primes = [];
theta = linspace(0,6*pi, 100);

figure;
for i=1:length(theta)
    H= expm(theta(i)*xi_hat);
    x_prime = H*x;
    x_primes =  [ x_primes, x_prime];


% plot the scene
clf;
plotPluckerLine( L, 80, 'b'); hold on; grid on; axis equal;
plot3(x_primes(1,:), x_primes(2,:), x_primes(3,:),'r','linewidth',4);
plot3(x_prime(1), x_prime(2), x_prime(3), 'ko','markersize',10,'markerfacecolor','k');
axis([-30 30 -30 30 -30 30 ]); drawnow; pause(0.005);
end
