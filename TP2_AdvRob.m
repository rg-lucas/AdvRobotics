clear; close all; clc;

% Point A:
% define initial theta:
theta_0 = [-pi/4;0;-pi/2;0;-pi/2;0];
theta = theta_0;
% Calculate initial pose from theta
[H0] = AdeptViperKinematicModels( theta_0 );
% Get point A from initial pose
A = H0(1:3,4);

% Point B:
% Define random point B

%B = ([0.4*(rand(1,1)+1); 0.4*(rand(1,1)+1); 0.4*(rand(1,1)+1)] - [-0.288-0.08;  0;      0.0645-.08-.26-.203   ])
B = [0;.1;-0.0645-.08-.26-.203]
n = 10;

%% Checking if the coordinates x, y aren't outside of the robots reach
%% Using a simplified checking process.

A_test = A -  [-0.288-0.08;  0;      0.0645-.08-.26-.203   ]
B_test = B -  [-0.288-0.08;  0;      0.0645-.08-.26-.203   ]

x = [A(1), B(1)];
y = [A(2), B(2)];
z = [A(3), B(3)];

%% Create the interpolations and the values of each point

New_x = linspace(x(1),x(2),n+2); % Includes given points
New_y = linspace(y(1),y(2),n+2); % Includes given points
New_z = linspace(z(1),z(2),n+2); % Includes given points
New_x(end)=[];
New_x(1)=[];
New_y(end)=[];
New_y(1)=[];
New_z(end)=[];
New_z(1)=[];

%% interpolated points for the robot pass through


p_inter = [A(1), New_x, B(1); A(2), New_y, B(2); A(3), New_z, B(3)]


% control loop parameters
eps = 10^(1); % epsilon for loop break (meters/second)
dt = 0.005; % control sampling time (seconds)
lambda = 100; % control gain (choose it such that dt*lambda << 1)
mu = 0.1; % damping factor for the pseudo-inverse jacobian

% TRAJECTORY MEMORY VARIABLES (for visualization only)
jt = []; % joint trajectory
et = []; % end-effector position (no orientation)

for i = 1:+1:length(p_inter(1,:))
  point = p_inter(:,i); % get point
  Hd = point_to_pose(point); % convert point to pose
  % theta_desired = AdeptViperik(Hd, theta); %use inverse kinematics to get theta for desired pose

  if (i == length(p_inter(1,:)))
    eps = 10^(-3);
  endif

  do

  % READ JOINT ANGLES HERE FROM THE REAL ROBOT

  [H,J] = AdeptViperKinematicModels( theta ); % current end-effector pose
  He = inv(H)*Hd; % pose error
  xi_hat = logm(He); % twist in its homogenous form 4x4 from pose error
  xi = vex3D( xi_hat ); % extrect twist vector 6x1 from its homogenous form

  % control law
  % option 1 :
  % theta_dot = lambda*J'*inv(J*J' + mu*eye(6))*xi;
  % option 2 : (faster solving without the inverse function using backslash "\" )
  theta_dot = lambda*J'*((J*J' + mu*eye(6))\xi);

  % move robot (we use joint velocity control for the real robot.)
  theta = theta + dt*theta_dot; % we use simple integration for a simulated robot.
  jt = [ jt, theta ]; % keep in memory (for visualization purposes)
  et = [ et, H(1:3,4) ]; % keep in memory (for visualization purposes)

  disp(['norm(xi) ', num2str(norm(xi))]);

until( norm(xi) < eps )

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
    for j=1:size(p_inter,2)
      plot3( p_inter(1,j), p_inter(2,j), p_inter(3,j), 'ro', 'markersize', 3, 'markerfacecolor', 'g');
    end
    drawnow; i,

end
if (A_test(1) < 0.001*854.88 && A_test(1) > 0.001*241.34 &&
  A_test(2) < 0.001*854.88 && A_test(2) > -0.001*241.34 &&
  B_test(1) < 0.001*854.88 && B_test(1) > 0.001*241.34 &&
  B_test(2) < 0.001*854.88 && B_test(2) > -0.001*241.34 &&
  A_test(3) < 0.001*779.88 && A_test(3) > -0.001*166.34 &&
  B_test(3) < 0.001*779.88 && B_test(3) > -0.001*166.34)

  printf('the range is  valid in X, Y, Z\n')

else
  printf('the range is not valid in X,Y, Z\n')

end
