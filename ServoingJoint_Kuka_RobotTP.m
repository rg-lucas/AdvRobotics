clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial joint configuration for the Kuka
q = [ 10*deg2rad; 10*deg2rad; 10*deg2rad; 10*deg2rad; 10*deg2rad; 10*deg2rad; 10*deg2rad ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired joint configuration for the Kuka
q_d = [ 60*deg2rad; 50*deg2rad; 30*deg2rad; 20*deg2rad; 20*deg2rad; 30*deg2rad; 40*deg2rad ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% simulation parameters
time = 0;  % current time
tf = 1; % final time (seconds)
dt = 0.1; % control sampling time (seconds)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Joint Position Control Loop
figure;
plot_kuka_robot_FinalPoint(q_d)
%YOUR CODE START HERE
while (time < tf)
    % Calculate the error between current joint configuration and desired
    error = q_d - q;
    Kp = 5;
    % Calculate the control signal (change in joint positions)
    control_signal = Kp * error;

    % Update the joint positions using the control signal
    q = q + control_signal * dt;

    % Update the existing figure with the current joint configuration
    % Update the existing figure with the current joint configuration
    plot_kuka_robot(q, 1);
    axis([-1.2 1.2 -1.2 1.2 0 1.3]);
    view(60, 20);

    % Update time
    time = time + dt;


    % Pause for a short duration to control the simulation speed
    pause(dt);



end

% Plot the final robot configuration
figure('Visible', 'on');
clf;
plot_kuka_robot(q, 1);
axis([-1.2 1.2 -1.2 1.2 0 1.3]);
view(60, 20);

