%%% Lucas Resende Gomes - Homework 7 - Adv Robotics

%% Inverse Kinematics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Numerical solution %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;
%% An algorithm based on a twist from error %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% applied to Franka 7 dof robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% INITIALIZATION
% desired joint configuration to discover
theta_desired = 0.5*rand(7,1);
figure;
plot_frankaRobot( theta_desired );
title('Plot 1')
% given desired end-effector pose
[Hd,J] = FrankaKinematicModels( theta_desired );
% an initial guess which should be close to solution
theta_0 = theta_desired + deg2rad (10*rand (7,1));
figure;
plot_frankaRobot( theta_0 );
title('Plot 2')

theta= theta_0;
eps = 10^(-5); % meters/second

%LOOP
do
  %currentend-effector pose of the virtual robot
  [H,J] = FrankaKinematicModels ( theta );
  He = inv(H)*Hd; %pose error
  xi_hat = logm(He); % twist in its homogeneous form 4x4 from pose error
  xi = vex3D (xi_hat); % extract twist vector 6x1 from its homogeneus form

  lambda = 0.1; % damping factor for the pseudo-inverse jacobian
  % Update joint values using damped pseudo-inverse of the Jacobian
  % option 1
  % theta = theta + J'*inv(J*J' + lambda*eye(6))*xi);
  % option 2 : (faster solving without the inverse fuction)
  theta = theta + J'*((J*J' + lambda*eye(6))\xi);

  disp(['norm(xi) ',num2str(norm(xi))]);

until( norm(xi) < eps )
% END OF LOOP
disp('initial, final and desired joint values');
[ rad2deg(theta_0), rad2deg(theta), rad2deg(theta_desired) ];
disp('Final pose error');
He,
figure;
plot_frankaRobot( theta );
title('Plot 3')

theta
theta_0
theta_desired
