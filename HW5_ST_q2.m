% Camera calibration matrix (K)
focal_length = 800; % in pixels
optical_center = [320, 240]; % in pixels

K = [focal_length, 0, optical_center(1);
     0, focal_length, optical_center(2);
     0, 0, 1];

% Relative pose between cameras (rotation matrix and translation vector)
R = eye(3); % Identity rotation for simplicity
T = [1; 0; 0]; % Translation vector, adjust as needed

% Define the 3D point (M)
M = [1; 2; 3];

% Transformation matrices for both cameras
P1 = [R, -R*T];
P2 = [eye(3), -T]; % Fix the transformation matrix for the second camera

% Compute pixel coordinates using projection equations
m1_homogeneous = K * P1 * [M; 1];
m1_homogeneous = m1_homogeneous ./ m1_homogeneous(3); % Convert to inhomogeneous coordinates
m1 = m1_homogeneous(1:2);

m2_homogeneous = K * P2 * [M; 1];
m2_homogeneous = m2_homogeneous ./ m2_homogeneous(3); % Convert to inhomogeneous coordinates
m2 = m2_homogeneous(1:2);

% Plotting the 3D scene
figure;
hold on;

% Plotting optical centers
plot3(-T(1), -T(2), -T(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Optical center camera 1
text(-T(1), -T(2), -T(3), '  Optical Center Camera 1', 'Color', 'r', 'FontSize', 8);

plot3(0, 0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Optical center camera 2
text(0, 0, 0, '  Optical Center Camera 2', 'Color', 'b', 'FontSize', 8);

% Plotting 3D point (M)
plot3(M(1), M(2), M(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
text(M(1), M(2), M(3), '  3D Point (M)', 'Color', 'g', 'FontSize', 8);

% Plotting image planes
image_plane = [0, 0, 1;
               640, 0, 1;
               640, 480, 1;
               0, 480, 1;
               0, 0, 1];
Verify = K * P1
Verify2 = K * P2

% Calculando a transposta de image_plane
image_plane_transpose = image_plane'

image_plane_projected1 = K * P1 * image_plane_transpose;
plot3(image_plane_projected1(1, :), image_plane_projected1(2, :), image_plane_projected1(3, :), 'k-');

image_plane_projected2 = K * P2 * image_plane_transpose;
plot3(image_plane_projected2(1, :), image_plane_projected2(2, :), image_plane_projected2(3, :), 'k-');

% Plotting rays
ray_CM1 = [0, 0, 0, M(1) - T(1); 0, 0, 0, M(2) - T(2); 0, 0, 0, M(3) - T(3)];
plot3(ray_CM1(1, :), ray_CM1(2, :), ray_CM1(3, :), 'm--');

ray_CM2 = [0, 0, 0, M(1); 0, 0, 0, M(2); 0, 0, 0, M(3)];
plot3(ray_CM2(1, :), ray_CM2(2, :), ray_CM2(3, :), 'm--');

% Plotting projected points on image planes
plot(m1(1), m1(2), 'bx', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(m1(1), m1(2), '  Projected Point (m1)', 'Color', 'b', 'FontSize', 8);

plot(m2(1), m2(2), 'gx', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
text(m2(1), m2(2), '  Projected Point (m2)', 'Color', 'g', 'FontSize', 8);

hold off;
axis equal;
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Stereo Vision: 3D Scene with Two Cameras');

