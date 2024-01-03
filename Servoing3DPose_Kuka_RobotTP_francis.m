%% Code by Lucas Resende Gomes and Fatma Ltaief
%% TP2 - Exercise 1

clear; clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial joint configuration and end-effector pose for the Kuka
q = [ 50*deg2rad; 45*deg2rad; 30*deg2rad; 20*deg2rad; 10*deg2rad; 25*deg2rad; 15*deg2rad ];

dq_AR = fkm( q );
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );
X_AR = [ R_AR, t_AR ; 0 0 0 1];  % Cartesian pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired end-effector pose for the Kuka robot
% s-pace
u_BR = [ 0; 0; 1];        % u (orientation axis)
theta_BR = 30*deg2rad;    % theta (orientation angle)
t_BR = [ 0.2; 0.2; 0.8 ]; % position

% dq-space
dq_BR = uthetat2dq( u_BR, theta_BR, t_BR );  % dual quaternion

% C-space
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );
X_BR = [ R_BR, t_BR ; 0 0 0 1];  % Desired Cartesian space pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kp = 10; % Proportional gain for position control
Ko = 5; % Proportional gain for orientation control

% Define simulation parameters
time = 0;  % Current time
tf = 0.75;    % Final time (seconds)
dt = 0.01; % Control sampling time (seconds)

%% End-effector 3D Pose Control Loop
traces = [];
figure;
%% Tracking the Error - Creating the variables for the time and for the errors
%% To be seted in the error graph
errors = [];
time_array = [];
while(time < tf)

    dq_AB = fkm(q);
    [u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AB );
    X_AR = [R_AR, t_AR; 0 0 0 1];

    J_AB = jacobian(q);
    error_dq_AB = muldualpq( conjdualqsimple( dq_BR), dq_AB);
    [u_AB, theta_AB, R_AB, t_AB ] = dualq2uthetaRt( error_dq_AB );

    %% Control Law
    k = 5;
    v = -k * (R_AB)' * t_AB;
    w = -k * theta_AB * u_AB;
    control_law_AB = [v; w];

    %% Convert Control Law to the good frame
    T_BR = [R_BR, skew(t_BR) * R_BR; zeros(3, 3), R_BR];

    control_law_AR = T_BR * control_law_AB;

    control_law_joint = pinv( J_AB)* control_law_AR;
    % Store the current joint configuration as the previous one
    q_prev = q;


    q = q + dt*control_law_joint;
    time = time + dt;

    % Error (desired x actual joint configurations)
    error_joint = norm(q - q_prev);

    % Append error magnitude to the errors array
    errors = [errors, error_joint];
    time_array = [time_array, time];

    %% Plot the Scene

    clf; hold on;
    plot_kuka_robot(q, 0);
    plot_pose(X_AR, 'k');
    plot_pose(X_BR, 'r');
    axis([-1.2 1.2 -1.2 1.2 0 1.3]);
    view(60, 20);
    title(['Time: ', num2str(time)]);
    drawnow;

end

%% Plot of the Error x Time Graph
figure;
plot(time_array, errors);
xlabel('Time (s)');
ylabel('Error Magnitude');
title('Error in Joint Position Control vs. Time');
grid on;



