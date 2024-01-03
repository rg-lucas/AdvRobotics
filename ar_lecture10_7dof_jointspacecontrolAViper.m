clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Proportional Control %%%%%%%%%%%
theta_desired = 0.5*rand(6,1); % desired joint configuration
theta0 = theta_desired + deg2rad(90*rand(6,1)); % inital joint configuration
theta = theta0;
% control parameters
dt = 0.001; % control sampling time (seconds)
Kp = 100; % control gain
eps = .1; % Small treshold to stop the control loop
%% control loop
do
  %% robot's conf is measured through the motors encoders
  e = theta_desired - theta; % error in radians
  u = Kp*e; % control law
  %% simulate Franka 7 dof numericallt through simple integrations
  theta = theta + dt*u; %updates
  % draw the robot
  clf; plotAdeptViperKinematicModels( theta, 'FRANKA' ); drawnow;
until(rad2deg(max(abs(e))) < eps ) % break condition
%% end  of loop control
disp('initial, final, desired joint values');
[ rad2deg(theta0), rad2deg(theta), rad2deg(theta_desired) ]

