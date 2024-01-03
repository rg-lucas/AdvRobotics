clear; close all; clc;

X = [rand(3,1);1]; % a point in homogeneous coordinates

% a line in Plucker coordinates
a = rand(3,1); % a point in Euclidean coordinates
b = rand(3,1); % another point in Euclidean coordinates
L = pluckerLinefrom2points( a, b ); % Plucker vector
eL = EucNormalizationPluckerLine( L ); %normalized vector

%rotation angles
theta = linspace(0,3*pi/2);

X_primes = [];
for i=1:length(theta)
    H = rot3aboutaLine(theta(i),eL);
    X_prime = H*X;
    X_primes = [X_primes, X_prime];
end

figure;
plot3(X(1),X(2),X(3), 'ko','markersize',20,'markerfacecolor','k'); hold on; grid on;
plot3(X_primes(1,:),X_primes(2,:),X_primes(3,:), 'r','linewidth',4); % trajectory
plot3(X_prime(1),X_prime(2),X_prime(3), 'ro','markersize',20,'markerfacecolor','r'); %last point
plotPluckerLine(L,3,'b'); axis equal; axis([-2 2 -2 2 -2 2]);

