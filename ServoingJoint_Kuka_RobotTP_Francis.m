%% Code by Lucas Resende Gomes and Fatma Ltaief
%% TP2 - Exercise 1

clear; close; clc;
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
dt = 0.01; % control sampling time (seconds)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Joint Position Control Loop
figure;

errors = [];
time_array = [];

while(time < tf)

e = q-q_d;
%control law
k=5;
control_law=-k*e;
%move the robot
q=q+ dt*control_law;
time=time+dt;


%% Plot Scene
hold on; clf;
plot_kuka_robot( q, 1 );
plot_kuka_robot( q_d, 1 );
axis([-1.2 1.2 -1.2 1.2 0 1.3]);
view( 60, 20 );
drawnow

%% Tracking the Error - Creating the variables for the time and for the errors
%% To be seted in the error graph

time_array = [time_array, time];
errors = [errors, -e];

end
%% Plot of the Error x Time Graph
figure;
plot(time_array, errors);
xlabel('Time');
ylabel('Error');
title('Error x Time');
grid on;
