%% Advanced Robotics - Lecture 10 - Joint Control
%% Illustration on 2 dof
clear; close all; clc; % refresh startsWith
%% initial robot configuration
%% unit length link's points' positions [ base, tip]
joint1 = zeros (2,1); joint2 = [0;1]; tip = [0;2];
Link1 = [ joint1, joint2 ]; % We create the first link
Link2 = [ joint2, tip]; % We create the second link

%% inverse kinematics ( current joint configuration)
theta1 = rad2deg( atan2( joint2(2), joint2(1) ));
vector2 = tip - joint2;
theta2 = rad2deg( atan2( vector2(2), vector2(1) )) - theta1;
theta = [theta1; theta2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Proportional Control %%%%%
theta_desired = [30;45]; % desired angle (degrees)

% The target robot (for visualization purpose)
TargetP1 = expm( skew2D(deg2rad(theta_desired(1))) )*[1;0];
TargetRobotLink1= [zeros(2,1), TargetP1 ];
TargetP2 = TargetP1 + expm( skew2D(deg2rad(theta_desired(1) + theta_desired(2))) )*[1;0];
TargetRobotLink2= [TargetP1, TargetP2 ];
trajp1 = []; trajp2 = []; figure; %(for visualization purposes only)

% Control parameters
dt = 0.001; % control sampling time (seconds)
Kp = 100; % control gain (choose it such that dt*Kp <<1)
eps = 0.1; % small threshold to stop the control loop (degrees)

e = theta_desired - theta; % Control loop
while( abs(e) > eps )
  %% in simulation we assume that the robot's configuration
  %% is measured through the motor encoder, so we know theta!
  e = theta_desired - theta; % error
  u = Kp*e; % control law (= theta_dot)

  %% simulate the 2 dof robot nuerically through simple integration
  theta = theta + dt*u;
  %% forward kinematics (for visualization purposes only)
  % New joint position:
  p1 = expm( skew2D(deg2rad(theta(1))) )*[1;0];
  Link1 = [zeros(2,1), p1];
  p2 = p1 + expm( skew2D(deg2rad(theta(1) + theta(2))) )*[1;0];
  Link2 = [p1, p2];
  trajp1 = [trajp1, p1];
  trajp2 = [trajp2, p2];

  %% draw the robot
  clf; hold on;
  plot( TargetRobotLink1(1,:), TargetRobotLink1(2,:), 'g-', 'linewidth', 4);
  plot( TargetRobotLink2(1,:), TargetRobotLink2(2,:), 'g-', 'linewidth', 4);
  plot( trajp1(1,:), trajp1(2,:), 'r.', 'markersize', 8);
  plot( trajp2(1,:), trajp2(2,:), 'r.', 'markersize', 8);
  plot( Link1(1,:), Link1(2,:), 'bo-', 'linewidth', 4);
  plot( Link2(1,:), Link2(2,:), 'bo-', 'linewidth', 4);
  axis equal; axis([-2.5 2.5 0 2.5]);
  xlabel('X', 'fontsize', 20); ylabel('Y', 'fontsize', 20); grid on;
  title(['2 dof robot at ', num2str(theta(1)), 'º',', ', num2str(theta(2)), 'º'], 'fontsize', 20);
  drawnow;

end %break condition
%% end of control loop

