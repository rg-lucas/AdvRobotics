clear; close all; clc;

a = [1;0;0]; % a point in Euclidean coordinates
b = [2;0;0]; % another point in Euclidean coordinates
L1 = pluckerLinefrom2points(a,b);

a = [0;1;0];
b = [0;2;0];
L0 = pluckerLinefrom2points(a,b);
eL = EucNormalizationPluckerLine(L0);

theta = linspace (0,2*pi-2*pi/30,30);

L_primes = [];

for i=1:length(theta)
    % rotation transformation about a line with an angle
    H = rot3aboutaLine(theta(i), eL);
    R = H(1:3,1:3); t=H(1:3,4);

    H_bar = [       R,      zeros(3,3);
                skew(t)*R,      R       ];

    % rotate the point about the line
    L_prime = H_bar*L1;
    % stack up the trajectory
    L_primes = [ L_primes, L_prime ];
end

% plot the scene
figure;
plotPluckerLine( eL, 3, 'b'); % line of rotation
hold on; grid on; axis([ -2 2 -2 2 -2 2]); axis equal;

for i=1:length(theta)
    plotPluckerLine( L_primes(:,i), 1, [0.5,0.5,0.5]);
end

plotPluckerLine( L1, 1, 'k'); % initial line
plotPluckerLine( L_prime, 1, 'r' ); % final line
