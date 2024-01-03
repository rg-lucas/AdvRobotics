%% Advanced Robotics - Lecture 10 - Joint Control
%% Illustration on 1 dof
clear; close all; clc; % refresh startsWith
%% initial robot configuration
%% unit length link's points' positions [ base, tip]
base = zeros (2,1); tip = [0;1];
Link = [ base, tip ]; % link is alligned on the y-axis

%% inverse kinematics ( current joint configuration)
theta = rad2deg( atan2( tip(2), tip(1) ));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Proportional Control %%%%%
theta_desired = 30; % desired angle (degrees)
%% the target robot (for visualization purposes only)
TargetRobot = expm (skew2D(deg2rad(theta_desired)) )*[0 2; 0 0];
traj = []; figure; %(for visualization purposes only)

dt = 0.001; % control sampling time (seconds)
Kp = 100; % control gain (choose it such that dt*Kp <<1)
eps = 0.1; % small threshold to stop the control loop (degrees)

% Control loop
do
  %% in simulation we assume that the robot's configuration
  %% is measured through the motor encoder, so we know theta!

  e = theta_desired - theta; % error
  u = Kp*e; % control law (= theta_dot)

  %% simulate the 1 dof robot nuerically through simple integration
  theta = theta + dt*u;
  %% forward kinematics (for visualization purposes only)
  Link(:,2) = expm( skew2D(deg2rad(theta)) )*[1;0]; % new tip position
  traj = [ traj, Link(:,2) ]; % (for visualization purposes only)

  %% draw the robot
  clf; hold on;
  plot( TargetRobot(1,:), TargetRobot(2,:), 'g-', 'linewidth', 4);
  plot( traj(1,:), traj(2,:), 'r.', 'markersize', 8);
  plot( Link(1,:), Link(2,:), 'bo-', 'linewidth', 4);
  axis equal; axis([-1.5 1.5 0 1.5]);
  xlabel('X', 'fontsize', 20); ylabel('Y', 'fontsize', 20); grid on;
  title(['1 dof robot at ', num2str(theta), 'º'], 'fontsize', 20);
  drawnow;

until( abs(e) < eps ) %break condition
%% end of control loop

