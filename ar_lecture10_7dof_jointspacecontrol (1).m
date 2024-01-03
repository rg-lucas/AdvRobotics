clear; close all; clc;
%%%%%%%%%%%%%%
%% Proportional Control %%%%%
theta_desired = 0.5*rand(7,1); %desired joint configuration
theta0 = theta_desired + deg2rad(90*rand(7,1)); % initial joint configuration
theta = theta0;
%control parameters
dt = 0.001; % control sampling time (seconds)
Kp = 100; % control gain (choose it such that dt*Kp <<1)
eps = 0.1; % small threshold to stop the control loop (degrees)
%% control loop
e = theta_desired - theta;
while(rad2deg(max(abs(e))) > eps)
  %robot's configuration is measured through motors' encoders
  e = theta_desired - theta; %error in radians
  u = Kp*e; % control law (= theta_dot)
  %% simulate Franka 7 dof robot numerically through simple integration
  theta = theta + dt*u; % update joint configuration of the robot
  %draw the robot
  clf; plotFrankaKinematicModels( theta, 'FRANKA' ); drawnow;
end % break condition
%% end of control loop
disp('initial, final, desired joint values');
[rad2deg(theta0), rad2deg(theta), rad2deg(theta_desired) ]
