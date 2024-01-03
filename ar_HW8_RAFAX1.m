close all;

% Plotting RAFAX1
theta_guessed = theta+deg2rad(15*rand(7,1));
Hd_RAFAX1 = RAFAX1KinematicModels(theta);
xi_prime_RAFAX1 = RAFAX1ik(Hd_RAFAX1, theta_guessed)
%%%%%OK
