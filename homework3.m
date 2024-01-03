clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera Intrinsic Matrix  ?
% Set up a variable to convert degrees to radians
deg2rad = pi/180;

% Camera intrinsic parameters
fx = 5;  % Focal length in the x-direction
fy = 5;  % Focal length in the y-direction
uo = 10; % Horizontal image center coordinate
vo = 10; % Vertical image center coordinate


% Create the camera intrinsic matrix K
K = [fx, 0, uo;
     0, fy, vo;
     0, 0, 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera frame at A
% Orientation and position of the camera frame at A
u_AR = [1; 0; 0];         % Orientation axis
theta_AR = -90 * deg2rad; % Orientation angle
t_AR = [0.3; -1; 0.8];   % Position

% Calculate the camera's transformation
dq_AR = uthetat2dq(u_AR, theta_AR, t_AR);
[u_AR, theta_AR, R_AR, t_AR] = dualq2uthetaRt(dq_AR);
T_AR = [R_AR, t_AR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Scene frame at B
% Orientation and position of the scene frame at B
u_BR = [1; 0; 0];         % Orientation axis
theta_BR = 0 * deg2rad;   % Orientation angle
t_BR = [0.3; 0; 0.8];     % Position

% Calculate the scene frame's transformation
dq_BR = uthetat2dq(u_BR, theta_BR, t_BR);
[u_BR, theta_BR, R_BR, t_BR] = dualq2uthetaRt(dq_BR);
T_BR = [R_BR, t_BR; 0 0 0 1];

%%%%%%%%%% Scene Points %%%%%%%%%%
%%% Define Scene Points in the Scene Frame (P_B) %%%
% Define 3D scene points in the frame at B
P_B = [0.3 0 0.3;
      -0.3 0 -0.3;
      0.3 0 -0.3;
      -0.3 0 0.3];

%%% Transform Scene Points to Reference Frame %%%
% Transform scene points from the scene frame (B) to the reference frame (R)
P_B44 = [P_B, ones(4, 1)];
P_R44 = (T_BR * P_B44')';
P_R = P_R44(:, 1:3)';

%%% Transform Scene Points to Camera Frame %%%
% Transform scene points from the reference frame (R) to the camera frame (A)
P_A44 = (T_AR * P_R44')';
P_A = P_A44(:, 1:3)';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Image of the Scene
points_image = (K * P_A)';

%% Plot everything in the reference frame

figure;
subplot(1,2,1);
hold on;
plot_camera(T_AR, 'b');
plot_pose(T_BR, 'r');
p1=[0.9 0 1.4];
p2=[0.9 0 0.2];
p3=[-0.30 0 0.2];
p4=[-0.30 0 1.4];
plot_plane(p1, p2, p3, p4, 'k');
% Plot scene points in the reference frame (R)
plot3(P_R(1, 1), P_R(2, 1), P_R(3, 1), 'k.','MarkerSize',20); % Black dot
plot3(P_R(1, 2), P_R(2, 2), P_R(3, 2), 'b.','MarkerSize',20); % Blue dot
plot3(P_R(1, 3), P_R(2, 3), P_R(3, 3), 'r.','MarkerSize',20); % Red dot
plot3(P_R(1, 4), P_R(2, 4), P_R(3, 4), 'g.','MarkerSize',20); % Green dot
axis([-1.2 1.2 -1.2 1.2 0 1.4]);
view(50, 40);

%% Plot the image of the camera
subplot(1,2,2);
hold on;
box on;
% Displaying the captured dots
scatter(points_image(1, 1), points_image(1, 2), 70, 'k', 'filled'); % Black dot
scatter(points_image(2, 1), points_image(2, 2), 70, 'b', 'filled'); % Blue dot
scatter(points_image(3, 1), points_image(3, 2), 70, 'r', 'filled'); % Red dot
scatter(points_image(4, 1), points_image(4, 2), 70, 'g', 'filled'); % Green dot

axis i-j;
axis image;
xlabel('i');
ylabel('j');
axis([6 16 2 12]);
title('Camera View');

