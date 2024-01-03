clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera Intrinsic Matrix  ?



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera frame at A
u_AR = [ 1; 0; 0];        % u (orientation axis)
theta_AR = -90*deg2rad;   % theta (orientation angle)
t_AR = [ 0.3; 0.2; 0.8 ]; % position

dq_AR = uthetat2dq( u_AR, theta_AR, t_AR ); % Dual quaternion pose
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );

X_AR = [R_AR, t_AR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Scene frame at B  ?


%% Scene points in the scene frame ?


%% Scene points in the reference frame ?

%% Scene points in the camera frame ?

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Take Image of the Scene ?


figure; 
%% Plot everything in the reference frame
subplot(1,2,1); hold on; 
plot_camera( X_AR, 'b' );
% plot scene points
axis([-1.2 1.2 -1.2 1.2 0 1.4]);
view( 50, 40 ); 

%% Plot the image of the camera
subplot(1,2,2); hold on; box on;
% plot 
axis ij; axis image; xlabel('i'); ylabel('j');
axis([ 0 1024 0 1024 ]);
title('Camera View');

