clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial joint configuration for the Kuka
q_AR = [ 10*deg2rad; 10*deg2rad; 10*deg2rad; 10*deg2rad; 10*deg2rad; 10*deg2rad; 10*deg2rad ];
q = q_AR;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired joint configuration for the Kuka
qd_BR = [ 60*deg2rad; 50*deg2rad; 30*deg2rad; 20*deg2rad; 20*deg2rad; 30*deg2rad; 40*deg2rad ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% simulation parameters
time = 0;  % current time
tf = 1; % final time (seconds)
dt = 0.1; % control sampling time (seconds)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Joint Position Control Loop
figure;

% Inicialize a matriz para armazenar os erros em função do tempo
traces = [];
errors = [];


%YOUR CODE START HERE
while (time < tf)
    % Calculate the error between current joint configuration and desired
    dq_AR = fkm( q );
    [ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt(dq_AR)
    X_AR = [ R_AR, t_AR, ; 0 0 0 1]; %cartesian pode

    %%Jacobian
    J_AR = jacobian(q)


    error_dq_AB = muldualpq( conjdualqsimple( qd_BR ), dq_AR);
    [ u_AB, theta_AB, R_AB, t_AB] = dualq2uthetaRt (error_dq_AB);

    error_AB = [ R_AB, t_AB ; 0 0 0 1];
    %% Control Law
    k = 10;
    v = -k * (R_AB)' * t_AB;
    w = -k * theta_AB * u_AB;
    C_L_AB = [ v; w];

    %% Convert Control Law in the good frame
    T_BR = [ R_BR, skew(t_BR) * R_BR;
                zeros(3 ,3), R_BR ];
    C_L_AR = T_BR * C_L_AB;

    % Update the joint positions using the control signal
    q = q + C_L_AR * dt;

    % Adicione o erro atual à matriz de erros
    errors = [errors, norm(error)];

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

% Plot the errors over time
figure;
plot(0:dt:tf-dt, errors);
xlabel('Tempo (s)');
ylabel('Erro');
title('Erro ao longo do tempo');
