close all
clear
clc

% Example camera extrinsic parameters (rotation matrices and translation vectors)
R1 = eye(3);               % Identity matrix (no rotation for simplicity)
T1 = [0; 0; 0];            % Translation vector for Camera 1

% Assuming Cameras are not aligned, introducing some rotation for Camera 2
theta_x = pi/10;
theta_y = pi/8;
theta_z = pi/15;

R2 = [
    1,             0,              0;
    0, cos(theta_x), -sin(theta_x);
    0, sin(theta_x), cos(theta_x)
] * [
    cos(theta_y), 0, sin(theta_y);
    0,            1,             0;
    -sin(theta_y), 0, cos(theta_y)
] * [
    cos(theta_z), -sin(theta_z), 0;
    sin(theta_z), cos(theta_z), 0;
    0,            0,            1
];

T2 = [1; -1; 2];           % Translation vector for Camera 2

% Example camera intrinsic parameters (focal length, principal point)
fx1 = 1000;                % Focal length in pixels for Camera 1
fy1 = 1000;                % Focal length in pixels for Camera 1
cx1 = 320;                 % Principal point x-coordinate for Camera 1
cy1 = 240;                 % Principal point y-coordinate for Camera 1

fx2 = 1200;                % Focal length in pixels for Camera 2
fy2 = 1200;                % Focal length in pixels for Camera 2
cx2 = 300;                 % Principal point x-coordinate for Camera 2
cy2 = 200;                 % Principal point y-coordinate for Camera 2

% Example intrinsic matrices
K1 = [fx1, 0, cx1; 0, fy1, cy1; 0, 0, 1];  % Intrinsic matrix for Camera 1
K2 = [fx2, 0, cx2; 0, fy2, cy2; 0, 0, 1];  % Intrinsic matrix for Camera 2

% Camera matrix for Camera 1 (no rotation or translation yet)
P1 = K1 * [eye(3), zeros(3, 1)];           % [K1 | 0]

% Camera matrix for Camera 2 (considering rotation and translation)
P2 = K2 * [R2, T2];                        % [K2 | R2 T2]

% Example world coordinates of the estimated point M
M_world = [1; 1; 5];

% Call the function
[m_cam1, m_cam2] = triangulation_and_projection(M_world, R1, T1, R2, T2, K1, K2);

% Display the results
disp("2D Coordinates in Camera 1:");
disp(m_cam1);

disp("2D Coordinates in Camera 2:");
disp(m_cam2);

% Call the visualization function
visualize_scene(m_cam1, m_cam2, P1, P2, M_world, R1, T1, R2, T2);

