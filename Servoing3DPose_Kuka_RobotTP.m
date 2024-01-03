clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial joint configuration and end-effector pose for the Kuka
q = [ 50*deg2rad; 45*deg2rad; 30*deg2rad; 20*deg2rad; 10*deg2rad; 25*deg2rad; 15*deg2rad ];

dq_AR = fkm( q );
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );
X_AR = [ R_AR, t_AR ; 0 0 0 1];  % Cartesian pose
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired end-effector pose for the Kuka robot
% s-pace
u_BR = [ 0; 0; 1];        % u (orientation axis)
theta_BR = 30*deg2rad;    % theta (orientation angle)
t_BR = [ 0.2; 0.2; 0.8 ]; % position


% dq-space   
dq_BR = uthetat2dq( u_BR, theta_BR, t_BR );  % dual quaternion

% C-space
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );
X_BR = [ R_BR, t_BR ; 0 0 0 1];  % Cartesian space pose


%% Plot Scene
figure; 
hold on; clf;
plot_kuka_robot( q, 0 ); 
plot_pose( X_AR, 'k' );
plot_pose( X_BR, 'r' );
axis([-1.2 1.2 -1.2 1.2 0 1.3]);
view( 60, 20 ); 

