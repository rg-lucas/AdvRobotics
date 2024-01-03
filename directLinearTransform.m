%% Lucas And Gustavo - HW6 - CPA
clear
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Cameras' parameters
%% camera focal length
f = 0.1; % meters (along z-axis)

% pixel dimensions
sx = 0.0001; % pixel size along x-axis (meters)
sy = 0.0001; % pixel size along y-axis (meters)

%% image size sensor space
Width_pix = 640; % pixels (along x-axis)
Height_pix = 480; % pixels (along y-axis)

%% image size metric space
Width_m = Width_pix*sx; % meters (along x-axis)
Height_m = Height_pix*sy; % meters (along y-axis)

%% projection coordinates in the image plane - metric space
%% principal point (image center)
pu = Width_m/2; % (along x-axis)
pv = Height_m/2; % (along y-axis)

K = [ f/sx,  0,   pu/sx;
        0,  f/sy, pv/sy;
        0,   0,     1   ];


% Define M*
M_star = [2; 2; 1];
% Position of Camera 1
T1 = [0; 0; 0];
H1 = eye(4);

% Camera 1 Reference Vector (pointing to the global Z axis)
V1 = [1; 2; 3];
% Camera position 2
% Rotation of 30 degrees in relation to the global Z axis
theta = deg2rad(30);
rotation_matrix = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
T2 = rotation_matrix * [3; 0; 0.25];
H2 = [ rotation_matrix, T2; zeros(1,3), 1];

% Camera 2 Reference Vector (pointing to the global Z axis)
V2 = -[3; 1; -1];
% Initial chart settings
figure;
hold on;
grid on;
axis equal;
% Plot of Camera 1 and Reference Vector V1
plot3([T1(1), T1(1) + V1(1)], [T1(2), T1(2) + V1(2)], [T1(3), T1(3) + V1(3)], 'Color', 'b', 'LineWidth', 2);
scatter3(T1(1), T1(2), T1(3), 100, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
% Plot of Camera 2 and Reference Vector V2
plot3([T2(1), T2(1) + V2(1)], [T2(2), T2(2) + V2(2)], [T2(3), T2(3) + V2(3)], 'Color', 'r', 'LineWidth', 2);
scatter3(T2(1), T2(2), T2(3), 100, 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
% Plot of point M*
scatter3(M_star(1), M_star(2), M_star(3), 100, 'g', 'filled');
% labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Camera Positions and Reference Vectors');
legend( 'V1', 'Câmera 1',  'V2', 'Câmera 2', 'Ponto M*');
hold off;

%% Projections and camera POV

project1 = camPOV( H1, K, f, M_star );
project2 = camPOV( H2, K, f, M_star );

pov1 = figure;
plotCamPov (H1, K, f, project1);

pov2 = figure;
plotCamPov (H2, K, f, project2);


A(1,:) = transpose(project1)*K*H1(1:3,1:4);
A(2,:) = transpose(project2)*K*H2(1:3,1:4);


% SVD
[U,Sig,V] = svd(A);

x = V(end,:);

x = x / x(4);
x3d = x(1:3);
