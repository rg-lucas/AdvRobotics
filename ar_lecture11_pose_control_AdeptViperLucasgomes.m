%% FRANKA Robot Cartesian Space Pose Control %%
clear; close all; more off; clc;
%% An algorithm based on a twist from error %%%%

%% INITIALIZATION
% an initial configuration of the robot
theta_0 = zeros(6,1);
theta = theta_0;

% desired end-effector pose to converge
theta_desired = theta + deg2rad(35)*rand(6,1);
[Hd,J] = AdeptViperKinematicModels( theta_desired );

% control loop parameters
eps = 10^(-2); % epsilon for loop break (meters/second)
dt = 0.006; % control sampling time (seconds)
lambda = 90; % control gain (choose it such that dt*lambda << 1)
mu = 0.1; % damping factor for the pseudo-inverse jacobian

% TRAJECTORY MEMORY VARIABLES (for visualization only)
jt = []; % joint trajectory
et = []; % end-effector position (no orientation)

% LOOP
[H,J] = AdeptViperKinematicModels( theta ); % current end-effector pose
He = inv(H)*Hd;
xi_hat = logm(He); % twist in its homogenous form 4x4 from pose error
xi = vex3D( xi_hat ); % extrect twist vector 6x1 from its homogenous form
while( norm(xi) > eps )

  % READ JOINT ANGLES HERE FROM THE REAL ROBOT

  [H,J] = AdeptViperKinematicModels( theta ); % current end-effector pose
  He = inv(H)*Hd; % pose error
  xi_hat = logm(He); % twist in its homogenous form 4x4 from pose error
  xi = vex3D( xi_hat ); % extrect twist vector 6x1 from its homogenous form

  % control law
  % option 1 : theta_dot = lambda*J'*inv(J*J' + mu*eye(6))*xi;
  % option 2 : (faster solving without the inverse function using backslash "\" )
  theta_dot = lambda*J'*((J*J' + mu*eye(6))\xi);

  % move robot (we use joint velocity control for the real robot.)
  theta = theta + dt*theta_dot; % we use simple integration for a simulated robot.
  jt = [ jt, theta ]; % keep in memory (for visualization purposes)
  et = [ et, H(1:3,4) ]; % keep in memory (for visualization purposes)

  disp(['norm(xi) ', num2str(norm(xi))]);

end


disp('Final pose error');
He,


%% VISUALIZATION
figure;
for i=1:length(jt)

    clf;
    plotAdeptViperKinematicModels( jt(:,i), "Adept Viper" );
    plotCoordinateFramefromPose( Hd, 0.1 );
    plot3(et(1,1:i),et(2,1:i),et(3,1:i),'b','linewidth',4);
    drawnow; i,

end
