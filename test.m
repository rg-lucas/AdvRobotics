% Complex plane analysis
% Rotating a line about a pivot point
clear; close all; clc;

z0 = 1 + i; % line with closest point
p = -1 +i*0.5; % pivot point

% Translate the line
% such that pivot point is at the origin
z = z0 + (z0 / (norm(z0)^2)) * real(conj(z0)*(-p));

% Rotate the line about the origin by phi
phi = pi/2;
z = exp(i*phi)*z;

% Tranlate the line such that pivot point
% returns to its initial position
z = z + (z / (norm(z)^2)) * real(conj(z)*p);
