close all
clear
clc

% Define a set of 3D points
points_3d = [1, 2, 3;
             4, 5, 6;
             7, 8, 9;
             10, 11, 12;
             13, 14, 15];

% Define a hypothetical camera calibration matrix (K)
focal_length = 800; % in pixels
optical_center = [320, 240]; % in pixels

K = [focal_length, 0, optical_center(1);
     0, focal_length, optical_center(2);
     0, 0, 1];

% Define a hypothetical relative pose between the two cameras
% (rotation matrix and translation vector)
R = eye(3); % Identity rotation for simplicity
T = [1; 0; 0]; % Translation vector, adjust as needed

% Project 3D points into two images
P1 = [R, -R * T];
P2 = [eye(3), -T]; % Second camera at the origin

points_3d_homogeneous = [points_3d, ones(size(points_3d, 1), 1)];

% Project points into image planes
points_image1_homogeneous = K * P1 * points_3d_homogeneous';
points_image1 = points_image1_homogeneous(1:2, :)' ./ points_image1_homogeneous(3, :)';

points_image2_homogeneous = K * P2 * points_3d_homogeneous';
points_image2 = points_image2_homogeneous(1:2, :)' ./ points_image2_homogeneous(3, :)';

% Create synthetic correspondences
correspondences = [points_image1, points_image2];

% Run the reconstruction algorithm
reconstructed_points = reconstruct_3d(correspondences, K);

% Animation loop
num_frames = 50;  % You can adjust the number of frames
figure;

for frame = 1:num_frames
    % Interpolate between original and reconstructed points
    interpolated_points = points_3d + (reconstructed_points - points_3d) * frame / num_frames;

    % Plot the original points
    subplot(1, 2, 1);
    plot3(points_3d(:, 1), points_3d(:, 2), points_3d(:, 3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    text(points_3d(:, 1), points_3d(:, 2), points_3d(:, 3), '  Original Points', 'Color', 'b', 'FontSize', 8);
    hold on;

    % Plot the lines
    plot3([interpolated_points(:, 1), reconstructed_points(:, 1)]', [interpolated_points(:, 2), reconstructed_points(:, 2)]', [interpolated_points(:, 3), reconstructed_points(:, 3)]', 'k--');

    % Plot the reconstructed points
    subplot(1, 2, 2);
    plot3(NaN, NaN, NaN, 'rx', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text(reconstructed_points(:, 1), reconstructed_points(:, 2), reconstructed_points(:, 3), '  Reconstructed Points', 'Color', 'r', 'FontSize', 8);
    hold on;
    plot3(interpolated_points(:, 1), interpolated_points(:, 2), interpolated_points(:, 3), 'r.');

    % Pause for a short duration to visualize the animation
    pause(0.1);

    % Clear the plots for the next frame
    clf;
end

