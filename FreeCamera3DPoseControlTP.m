Eclear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Camera Pose at A wrt to the fixed frame R
% s-pace
u_AR = [ 1; 0; 0];          % u (orientation axis)
theta_AR = -60*deg2rad;     % theta (orientation angle)
t_AR = [ -0.1; -0.1; 0.9 ]; % position

% dq-space
dq_AR = uthetat2dq( u_AR, theta_AR, t_AR );  % dual quaternion

% C-space
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );
X_AR = [ R_AR, t_AR ; 0 0 0 1];  % Cartesian space pose


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Camera Pose at B wrt to the fixed frame R
% s-pace
u_BR = [ 1; 0; 0];         % u (orientation axis)
theta_BR = -90*deg2rad;    % theta (orientation angle)
t_BR = [ 0.3; 0.2; 0.8 ];  % position

% dq-space
dq_BR = uthetat2dq( u_BR, theta_BR, t_BR );  % dual quaternion

% C-space
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );
X_BR = [ R_BR, t_BR ; 0 0 0 1];  % Cartesian space pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
traces = []; % the trajectory of the camera frame

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 0;  % current time
tf = 0.6; % final time
dt = 0.001; % control sampling time

figure;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while(time < tf)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%VOTRE CODE ICI
%% Compute error : you can use dual quaternion functions first
error_d
%% Control Law


%% Convert Control Law in the good frame


%% Move Camera

%FIN DE VOTRE CODE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Scene
time = time + dt;
traces = [ traces, t_AR ]; % the trajectory of the camera frame
clf; hold on;
plot3( traces(1,:), traces(2,:), traces(3,:), 'k' );

plot_pose( X_AR, 'k'  );  plot_camera( X_AR, 'b' );
plot_pose( X_BR, 'k'  );  plot_camera( X_BR, 'r' );

axis([-1.2 1.2 -1.2 1.2 0 1.4]);
view( 64, 34 );
title( num2str(time) );
drawnow;

end
