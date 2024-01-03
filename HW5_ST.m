% Camera calibration matrix (K)
focal_length = 800; % in pixels
optical_center = [320, 240]; % in pixels

K = [focal_length, 0, optical_center(1);
     0, focal_length, optical_center(2);
     0, 0, 1];

% Camera center (C)
C = [0; 0; 0];

% 3D point coordinates (M)
M = [1; 2; 3];

% Compute pixel coordinates using perspective projection
m_homogeneous = K * [eye(3), -C] * [M; 1];
m_homogeneous = m_homogeneous ./ m_homogeneous(3); % Convert to inhomogeneous coordinates
m = m_homogeneous(1:2);

% Plotting the 3D scene
figure;
hold on;

% Plotting optical center
plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(0, 0, 0, '  Optical Center', 'Color', 'r', 'FontSize', 8);

% Plotting 3D point (M)
plot3(M(1), M(2), M(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
text(M(1), M(2), M(3), '  3D Point (M)', 'Color', 'g', 'FontSize', 8);

% Plotting camera center (C)
plot3(C(1), C(2), C(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(C(1), C(2), C(3), '  Camera Center (C)', 'Color', 'b', 'FontSize', 8);

% Plotting image plane
image_plane = [0, 0, 1;
               640, 0, 1;
               640, 480, 1;
               0, 480, 1;
               0, 0, 1];
image_plane_projected = K * image_plane';

plot3(image_plane_projected(1, :), image_plane_projected(2, :), image_plane_projected(3, :), 'k-');
text(optical_center(1) + 20, optical_center(2) + 20, 1, 'Image Plane', 'Color', 'k', 'FontSize', 8);

% Plotting ray CM
ray_CM = [C, M];
plot3(ray_CM(1, :), ray_CM(2, :), ray_CM(3, :), 'm--');
text(M(1), M(2), M(3), '  Ray CM', 'Color', 'm', 'FontSize', 8);

% Plotting projected point (m) on the image plane
plot(m(1), m(2), 'bx', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(m(1), m(2), '  Projected Point (m)', 'Color', 'b', 'FontSize', 8);

hold off;
axis equal;
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D Scene with Camera Calibration and Projection');

% Display the image plane
figure;
plot(image_plane(:,1), image_plane(:,2), 'k-');
hold on;
plot(m(1), m(2), 'bx', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
hold off;
axis equal;
grid on;
xlabel('X-axis (Pixels)');
ylabel('Y-axis (Pixels)');
title('Image Plane with Projected Point (m)');

