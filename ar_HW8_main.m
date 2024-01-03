close all;

theta = deg2rad(15*rand(7,1));


##% Plotting UR10
##theta_guessed = theta+deg2rad(15*rand(7,1));
##Hd_UR10 = UR10KinematicModels(theta);
##xi_prime_UR10 = UR10ik(Hd_UR10, theta_guessed)
##%%%%%OK
##
##% Plotting AdeptViper
##theta_viper = theta(1:6);
##theta_guessed = theta_viper+deg2rad(10*rand(6,1));
##Hd_AdeptViper = AdeptViperKinematicModels(theta_viper);
##xi_prime_AdeptViper = AdeptViperik(Hd_AdeptViper, theta_guessed)
##%%%%%OK
##
##% Plotting RAFAX1
##theta_guessed = theta+deg2rad(10*rand(7,1));
##Hd_RAFAX1 = RAFAX1KinematicModels(theta);
##xi_prime_RAFAX1 = RAFAX1ik(Hd_RAFAX1, theta_guessed)
##%%%%%OK


% Plotting KUKALWR4
theta_guessed = theta+deg2rad(15*rand(7,1));
Hd_KukaLWR4 = KukaLWR4KinematicModels(theta);
xi_prime_KukaLWR4 = KukaLWR4ik(Hd_KukaLWR4, theta_guessed)
%%%%%OK




