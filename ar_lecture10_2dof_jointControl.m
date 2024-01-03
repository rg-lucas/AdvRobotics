%% Advanced Robotics - Lecture 10 - Joint Control
%% Illustration on 1 dof
clear; close all; clc; % a refresh start
%% initial robot configuration
%% unit lenght link's points' position [ base, tip ]
base = zeros(2,1); joint = [1; 0]; tip = [1; 1];
Link1 = [base, joint ]; % link is aligned on the y-axis
Link2 = [joint, tip ]; % link is aligned on the y-axis

theta_desired = [120; 135];
%% inverse kinematics (current joint configuration)
theta = [rad2deg( atan2( tip(2), tip(1)) ); rad2deg( atan2(tip(2), tip(1)) - atan2( joint(2), joint(1)) )];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Proportional Control
traj = []; figure; % (for visualization purposes only)

dt = 0.001; % control sampling time (seconds)
Kp = 100; % control gain (choose it such that dt*Kp << 1 )
eps = 0.1; % small threshold to stop the control loop (degrees)

%% Control loop
do
 %% in simulation we assume that the robot's configuration
 %% is measured through the motor encoder, so we know theta!

 e = theta_desired - theta; % error
 u = Kp*e; % control law (= theta_dot)

 %% simulate the 1 dof robot numericaly through simple integration
 theta = theta + dt*u; % update configuration of the robot
%% foward kinematics (for visualization purposes only)
Link1 (:,2) = expm(skew2D(deg2rad(theta(1))) )*[1;0]; % new top position
Link2 (:,2) = expm(skew2D(deg2rad(theta(2))) )*[1;1]; % new top position
Link2(:,1) = Link1(:,2);
traj = [ traj, [Link1(:,2); Link2(:,2)] ]; % (for visualization purposes only)

%% draw the robot
clf; hold on;
plot( traj(1,:), traj(2,:), 'r.', 'markersize', 8 );
plot( traj(3,:), traj(4,:), 'r.', 'markersize', 8 );
plot( Link1(1,:), Link1(2,:), 'bo-', 'linewidth', 4 );
plot( Link2(1,:), Link2(2,:), 'bo-', 'linewidth', 4 );
axis equal; axis([-1.5 1.5 0 1.5]);
xlabel('X', 'fontsize', 20); ylabel('Y', 'fontsize', 20); grid on;
drawnow;

until( abs(e) < eps ) % break condition
%% end of control loop












