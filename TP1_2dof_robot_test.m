%% TP1_2dof_robot test

%% Advanced Robotics - Lecture 10 - Joint Control
%% Illustration on 2 dof
clear; close all; clc; % refresh startsWith
%% initial robot configuration

%% inverse kinematics ( current joint configuration)
theta0 = [pi;pi];
[q0, H0] = TwoDof_KinematicModels(theta0);

theta1 = rad2deg( atan2( q0(2), q0(1) ));
vector2 = [q0(3);q0(4)] - [q0(1);q0(2)];
theta2 = rad2deg( atan2( vector2(2), vector2(1) )) - theta1;
theta = [theta1; theta2];

lenL1 = hypot(q0(2), q0(1));% We create the first link
lenL2 = hypot(vector2(2), vector2(1));% We create the second link

H0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test of Proportional Control %%%%%
theta_desired = [30;45]; % desired angle (degrees)

% The target robot (for visualization purpose)
[q_tgt, H_tgt] = TwoDof_KinematicModels(theta_desired);

TargetP1 = q_tgt(1:2);
TargetRobotLink1= [zeros(2,1), TargetP1 ];
TargetP2 = q_tgt(3:4);
TargetRobotLink2= [TargetP1, TargetP2 ];
H_tgt;

trajp1 = []; trajp2 = []; figure; %(for visualization purposes only)

theta_desired = ik_2dof(H_tgt(1), H_tgt(2), lenL1, lenL2)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Now we start: %%%%%%%%%%%%%%%%

%% create 2 points and check if they are in the range of the robot  or not.

A = 1.4*rand(1,2);
B = [-1.3*rand(1,1), 1.3*rand(1,1)];

%% Check the distance of those points to the base of the robot
C = sqrt(A(1)^2+A(2)^2);
D = sqrt(B(1)^2+B(2)^2);


%% Give the size of the robot to check if the distance is reachable
robotSize = 1.5;
%% Checking if A and B are in the range of the robot
if C < robotSize && D < robotSize
  printf('the range is  valid')

else
  printf('the range is not valid')

end


x = [A(1), B(1)];
y = [A(2), B(2)];
n = 5;

%% Create the interpolations and the values of each point

New_x = linspace(x(1),x(2),n+2); % Includes given points
New_y = linspace(y(1),y(2),n+2); % Includes given points
New_x(end)=[];
New_x(1)=[];
New_y(end)=[];
New_y(1)=[];

%% interpolated points for the robot pass through
p_inter = [A(1), New_x, B(1); A(2), New_y, B(2)];

theta0 = ik_2dof(p_inter(1,1), p_inter(2,1), lenL1, lenL2);
theta = theta0;


% Control parameters
dt = 0.001; % control sampling time (seconds)
Kp = 100; % control gain (choose it such that dt*Kp <<1)
eps = 100; % small threshold to stop the control loop (degrees)


for i = 1:+1:length(p_inter(1,:))
  theta_desired = ik_2dof(p_inter(1,i), p_inter(2,i), lenL1, lenL2);
  [q_tgt, H_tgt] = TwoDof_KinematicModels(theta_desired);

  TargetP1 = q_tgt(1:2);
  TargetRobotLink1= [zeros(2,1), TargetP1 ];
  TargetP2 = q_tgt(3:4);
  TargetRobotLink2= [TargetP1, TargetP2 ];

  e = theta_desired - theta; % Control loop
  if i = length(p_inter(1,:))
    eps = eps/100;
  end

while( abs(e) > eps )
  %% in simulation we assume that the robot's configuration
  %% is measured through the motor encoder, so we know theta!
  e = theta_desired - theta; % error
  u = Kp*e; % control law (= theta_dot)

  %% simulate the 2 dof robot nuerically through simple integration
  theta = theta + dt*u;
  %% forward kinematics (for visualization purposes only)
  % New joint position:
  q = TwoDof_KinematicModels(theta);
  p1 = q(1:2);
  Link1 = [zeros(2,1), p1];
  p2 = q(3:4);
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
  axis equal; axis([-2 2 0 2]);
  xlabel('X', 'fontsize', 20); ylabel('Y', 'fontsize', 20); grid on;
  title(['2 dof robot at ', num2str(theta(1)), 'º',', ', num2str(theta(2)), 'º'], 'fontsize', 20);
   plot( A(1),A(2), 'b', 'markersize', 20);
   plot( B(1),B(2), 'g', 'markersize', 20);
   plot(x,y);
   scatter(New_x,New_y,'r','filled');
   axis equal; axis([-2.5 2.5 0 2.5]);
   xlabel('X', 'fontsize', 20); ylabel('Y', 'fontsize', 20); grid on;
  drawnow;

end %break condition
%% end of control loop
end


