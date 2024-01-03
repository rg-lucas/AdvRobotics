clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera Intrinsic Matrix
fx = 800; % focal length (mm) / pixel_width_x (mm) = pixels
fy = 800; % focal length (mm) / pixel_height_y (mm) = pixels
uo = 512; % image center x-coordinate
vo = 512; % image center y-coordinate

K = [ fx,  0, uo;
       0, fy, vo;
       0,  0,  1 ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Current Camera location at A wrt to the fixed reference frame at R
u_AR = [ 1; 0; 0];         % u (orientation axis)
theta_AR = -110*deg2rad;   % theta (orientation angle)
t_AR = [ 0.0; 0.0; 0.9 ];  % position

dq_AR = uthetat2dq( u_AR, theta_AR, t_AR ); % Dual quaternion pose
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );

X_AR = [ R_AR, t_AR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Camera location at B wrt to the fixed reference frame at R
u_BR = [ 1; 0; 0];         % u (orientation axis)
theta_BR = -90*deg2rad;    % theta (orientation angle)
t_BR = [ 0.3; 0.2; 0.8 ]; % position

dq_BR = uthetat2dq( u_BR, theta_BR, t_BR ); % Dual quaternion pose
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );

X_BR = [ R_BR, t_BR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pattern location at C wrt to the fixed reference frame at R
u_CR = [ 1; 0; 0];        % u (orientation axis)
theta_CR = 90*deg2rad;    % theta (orientation angle)
t_CR = [ 0.2; 0.9; 0.7 ]; % position

dq_CR = uthetat2dq( u_CR, theta_CR, t_CR ); % Dual quaternion pose
[ u_CR, theta_CR, R_CR, t_CR ] = dualq2uthetaRt( dq_CR );


X_CR = [ R_CR, t_CR; 0 0 0 1];

pattern_points_CR = put_pattern( X_CR ); % 3D pattern points

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Image of the Pattern from location B ?

dq_RB = conjdualqsimple(dq_BR);
[u_RB, theta_RB, R_RB, t_RB ] = dualq2uthetaRt(dq_RB);
X_RB = [R_RB, t_RB; 0 0 0 1]

image_pattern_pix_B = take_image( X_RB, K, pattern_points_CR)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired image point features vector s_B from location B in metric units ?
image_pattern_metreic_B = pattern_pix2metric( image_pattern_pix_B, fx, fy, uo, vo);
s_B = image_pattern_mettric_B(:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
traces = []

time = 0;
tf = 6;
Hz = 50;
dt = 1/50;

figure
%% LOOP SHOULD START FROM HERE
while (time < tf)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Take an image from location A ?
  dq_RA = conjdualqsimple(dq_AR);
  [ u_RA, theta


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Build the point features vector s_A in metric units ?

%% Calculate interaction matrix (Image Jacobian) ?

%% Error ?

%% Compute Control Law ?

%% Convert Control Law ?

%% Move Camera ?

%% Plot the 3D scene wrt to the reference frame at R
figure;
subplot(1,2,1); hold on;
plot_pattern( pattern_points_CR );
plot_camera( X_AR, 'b' );
plot_camera( X_BR, 'r' );
axis([-1.2 1.2 -1.2 1.2 0 1.4]);
view( 50, 40 );

%% Plot the image of the camera from locations A and B
subplot(1,2,2); hold on; box on;
axis ij; axis image; xlabel('i'); ylabel('j');
axis([ 0 1024 0 1024 ]);
title('Camera View');
