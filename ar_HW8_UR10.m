close all;

% Plotting UR10
theta = deg2rad(15*rand(7,1));
theta_guessed = theta+deg2rad(15*rand(7,1));
Hd_UR10 = UR10KinematicModels(theta);
xi_prime_UR10 = UR10ik(Hd_UR10, theta_guessed)
%%%%%OK









