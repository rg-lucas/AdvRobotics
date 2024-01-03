clear;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera Intrinsic Matrix
fx = 800; % Focal length (mm) / pixel_width_x (mm) = pixels
fy = 800; % Focal length (mm) / pixel_height_y (mm) = pixels
uo = 512; % Image center x-coordinate
vo = 512; % Image center y-coordinate

K = [fx, 0, uo;
     0, fy, vo;
     0, 0, 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Current Camera location at A wrt to the fixed reference frame at R
u_AR = [1; 0; 0];         % u (orientation axis)
theta_AR = -110 * deg2rad;   % theta (orientation angle)
t_AR = [0.0; 0.0; 0.9];  % position

dq_AR = uthetat2dq(u_AR, theta_AR, t_AR); % Dual quaternion pose
[u_AR, theta_AR, R_AR, t_AR] = dualq2uthetaRt(dq_AR);

X_AR = [R_AR, t_AR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Camera location at B wrt to the fixed reference frame at R
u_BR = [1; 0; 0];         % u (orientation axis)
theta_BR = -90 * deg2rad;    % theta (orientation angle)
t_BR = [0.3; 0.2; 0.8]; % position

dq_BR = uthetat2dq(u_BR, theta_BR, t_BR); % Dual quaternion pose
[u_BR, theta_BR, R_BR, t_BR] = dualq2uthetaRt(dq_BR);

X_BR = [R_BR, t_BR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pattern location at C wrt to the fixed reference frame at R
u_CR = [1; 0; 0];        % u (orientation axis)
theta_CR = 90 * deg2rad;    % theta (orientation angle)
t_CR = [0.2; 0.9; 0.7]; % position

dq_CR = uthetat2dq(u_CR, theta_CR, t_CR); % Dual quaternion pose
[u_CR, theta_CR, R_CR, t_CR] = dualq2uthetaRt(dq_CR);

X_CR = [R_CR, t_CR; 0 0 0 1];

pattern_points_CR = put_pattern(X_CR); % 3D pattern points

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Image of the Pattern from location B ?

dq_RB = conjdualqsimple(dq_BR);
[u_RB, theta_RB, R_RB, t_RB] = dualq2uthetaRt(dq_RB);
X_RB = [R_RB, t_RB; 0 0 0 1];

% Simulate the image of the pattern from location B
image_pattern_pix_B = take_image(X_RB, K, pattern_points_CR);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired image point features vector s_B from location B in metric units ?
image_pattern_metric_B = pattern_pix2metric(image_pattern_pix_B, fx, fy, uo, vo);
s_B = image_pattern_metric_B(:);

traces = [];

time = 0;
tf = 6;
Hz = 50;
dt = 1 / Hz;

% Simulation loop
while time < tf
    % Capture an image from location A (Assuming this operation)
    % Calculate interaction matrix (Image Jacobian)
    % Calculate error
    % Compute control law
    % Move camera (Assuming this operation)

    % Store camera pose
    traces = [traces; t_AR.'];

    time = time + dt;
end

% Plot the camera movement in the reference frame
figure;
plot3(traces(:, 1), traces(:, 2), traces(:, 3), 'b.-');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
title('Camera Movement in the Reference Frame');

function image_pattern_pix_B = take_image(X_RB, K, pattern_points_CR)
    % This function simulates capturing the pattern from location B

    % Assuming this operation (replace with actual image capture code)
    % You can use MATLAB's camera projection functions here
    t = X_RB(1:3, 4);
    R = X_RB(1:3, 1:3);

    % Transform pattern points from world frame to camera frame
    pattern_points_CB = (R' * pattern_points_CR' + t)';

    % Project 3D points to 2D image coordinates using the intrinsic matrix
    pattern_points_pix_B = (K * pattern_points_CB')';

    % Create the image of the pattern at location B
    image_pattern_pix_B = zeros(1024, 1024);

    for i = 1:size(pattern_points_pix_B, 1)
        x = pattern_points_pix_B(i, 1);
        y = pattern_points_pix_B(i, 2);

        if x > 0 && x <= 1024 && y > 0 && y <= 1024
            image_pattern_pix_B(round(y), round(x)) = 1; % Mark the points
        end
    end

    % Apply some blurring or other transformations for a more realistic image
end

function image_pattern_metric_B = pattern_pix2metric(image_pattern_pix_B, fx, fy, uo, vo)
    % Convert image coordinates to metric units using the camera intrinsic parameters
    [rows, cols] = size(image_pattern_pix_B);
    [X, Y] = meshgrid(1:cols, 1:rows);

    X = (X - uo) / fx;
    Y = (Y - vo) / fy;

    % Assuming depth values are all 1 (since this is an image of the pattern)
    Z = ones(rows, cols);

    image_pattern_metric_B = cat(3, X, Y, Z);
end


