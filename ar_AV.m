

% Plotting AdeptViper
theta = deg2rad(15*rand(6,1));
theta_guessed = theta_viper+deg2rad(10*rand(6,1));
Hd_AdeptViper = AdeptViperKinematicModels(theta_viper);
xi_prime_AdeptViper = AdeptViperik(Hd_AdeptViper, theta_guessed)
