clear; close all; clc;

% Defining Line A:
a0 = [.25;0;0]; % start of Line A in Euclidean coordinates
a1 = [2;0;0]; % end of Line A in Euclidean coordinates
A = pluckerLinefrom2points(a0,a1); % creates plucker line A
a0 = [a0;1]; % convert to homogeneous coordinates
a1 = [a1;1]; % convert to homogeneous coordinates

% Defining Line B:
b0 = [0;0;-2]; % start of Line B in Euclidean coordinates
b1 = [0;0;2]; % end of Line B in Euclidean coordinates
B = pluckerLinefrom2points(b0,b1); % creates plucker line B
eLB = EucNormalizationPluckerLine(B); % normalizes B to euclidean coordinates

% Defining point Q as the point that will turn around the lines:
Q = [1.25; .5; 0; 1]; % point in homogeneous coordinates


% create an array of angles that will be used as the step in the animation:
theta = linspace (0,2*pi-2*pi/180,180);

% Setting up space to record trajectory
LA_primes = [];
a0_primes = [];
a1_primes = [];
Q_primes = [];

for i=1:length(theta)
    % rotation transformation around a line with an angle
    H = rot3aboutaLine(theta(i), eLB);
    R = H(1:3,1:3); t=H(1:3,4);

    H_bar = [       R,      zeros(3,3);
                skew(t)*R,      R       ];

    % rotate A around B
    LA_prime = H_bar*A;
    a0_prime = H*a0;
    a1_prime = H*a1;

    % rotate Q around A around B
    eLA_prime = EucNormalizationPluckerLine(LA_prime);
    H_prime = rot3aboutaLine(theta(i)*12,eLA_prime);
    Q_prime = H_prime*H*Q;

    % stack up the trajectory
    LA_primes = [ LA_primes, LA_prime ];
    a0_primes = [ a0_primes, a0_prime ];
    a1_primes = [ a1_primes, a1_prime ];
    Q_primes = [Q_primes, Q_prime];

    % plotting the animation
    clf;  % clear the figure
    hold on; grid on; axis([ -2 2 -2 2 -2 2]); axis equal; view([2 2 2]); % set up view
    plotPluckerLineV2( eLB, b0, b1, [.5,0.5,.5]); % line B
    plotPluckerLineV2( LA_prime,a0_prime, a1_prime, [1,0.5,0]); % line A
    plot3(Q_prime(1),Q_prime(2),Q_prime(3), 'wo', 'markersize', 7, 'markerfacecolor', 'c'); % point Q

    pause(.0005);
end

for i=1:10:length(theta)
    plotPluckerLineV2( LA_primes(:,i),a0_primes(:,i), a1_primes(:,i), [1,0.5,0]);
    plot3(Q_primes(1,:),Q_primes(2,:),Q_primes(3,:), 'c','linewidth',4); % trajectory
end

plotPluckerLineV2( LA_prime, a0_prime, a1_prime, 'r' ); % final line
