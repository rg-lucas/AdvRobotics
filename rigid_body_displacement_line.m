% rigid-body displacement of a line
clear; close all; clc;
%hessian coordinates of a line
phi = deg2rad(45); d=1;
% homogeneous coordinates of a line
L = [cos(phi); sin(phi); -d];
nL = normalize_homogeneous_line(L);

%plot the line in black
plot2Dline(nL, [-2,2], 'k'); hold on; grid on;

% homogeneous transformation
theta = deg2rad(45); %rotation angle
R = expm(skew2D(theta) );
t = [1;1]; %translation vector
H = [ R, t; zeros(1,2), 1];

% transform the line
L_prime = inv(H')*nL; % inv function inverts matrix
nL_prime = normalize_homogeneous_line(L_prime),

% plot the transformed line in reducepatch
plot2Dline(nL_prime, [-2,2], 'r'); hold on; grid on;

