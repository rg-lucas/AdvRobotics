clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera Intrinsic Matrix
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

% Calculate the camera's transformation at A
dq_AR = uthetat2dq(u_AR, theta_AR, t_AR);
[u_AR, theta_AR, R_AR, t_AR] = dualq2uthetaRt(dq_AR);
T_AR = [R_AR, t_AR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Scene frame at B
% Orientation and position of the scene frame at B
u_BR = [1; 0; 0];         % Orientation axis
theta_BR = 0 * deg2rad;   % Orientation angle
t_BR = [0.3; 0; 0.8];     % Position

% Calculate the scene frame's transformation at B
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

%% Create an array to store intermediate camera transformations
num_steps = 100;  % Number of intermediate steps

intermediate_dq_AR = [];
intermediate_T_AR = cell(1, num_steps);

% Interpolate between initial and final camera transformations
for i = 1:num_steps
    alpha = i / num_steps;  % Interpolation parameter
    intermediate_dq_AR = slerp(dq_AR, dq_BR, alpha);
    [u_AR, theta_AR, R_AR, t_AR] = dualq2uthetaRt(intermediate_dq_AR);
    intermediate_T_AR{i} = [R_AR, t_AR; 0 0 0 1];
end

%% Plot the camera's motion
figure;
subplot(1, 2, 1);
hold on;
plot_camera(T_AR, 'b');  % Initial camera position
plot_pose(T_BR, 'r');    % Scene frame at B

% Plot the plane
p1 = [0.9 0 1.4];
p2 = [0.9 0 0.2];
p3 = [-0.30 0 0.2];
p4 = [-0.30 0 1.4];
plot_plane(p1, p2, p3, p4, 'k');

% Plot scene points in the reference frame
plot3(P_R(1, :), P_R(2, :), P_R(3, :), 'k.', 'MarkerSize', 20);

axis([-1.2 1.2 -1.2 1.2 0 1.4]);
view(50, 40);
title('Reference Frame');

%% Plot the camera's motion from A to B
subplot(1, 2, 2);
hold on;
box on;

for i = 1:num_steps
    intermediate_T = intermediate_T_AR{i};
    % Plot the camera frame at intermediate steps (blue)
    plot_camera(intermediate_T, 'b');
end

% Plot the scene frame at B (red)
plot_pose(T_BR, 'r');

% Plot the plane
plot_plane(p1, p2, p3, p4, 'k');

% Plot scene points in the reference frame
plot3(P_R(1, :), P_R(2, :), P_R(3, :), 'k.', 'MarkerSize', 20);

axis([-1.2 1.2 -1.2 1.2 0 1.4]);
view(50, 40);
title('Camera Motion from A to B');

