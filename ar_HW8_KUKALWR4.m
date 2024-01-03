close all;

% Plotting KUKALWR4
theta_guessed = theta+deg2rad(15*rand(7,1));
Hd_KukaLWR4 = KukaLWR4KinematicModels(theta);
xi_prime_KukaLWR4 = KukaLWR4ik(Hd_KukaLWR4, theta_guessed)
%%%%%OK
