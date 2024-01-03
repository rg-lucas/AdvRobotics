%% Hand-eye calibration %%%
%% Scenario : eye-in-hand
%% A camera is attached to the end-effector of the robot for a vision based task.
%% We need to compute where the camera is attached exactly on the robot
%% to be able to perform the task.
clear; close; more off; clc;

% Ground truth
X = randomH44( ); % unknown

% Initial configuration
E0 = randomH44( );
C0 = E0*X;

% Rigid-body motion 1 of the end-effector.
% This is known from forward kinematics of the robot
A1 = randomH44( );

% Next configuration under rigid-body motion 1.
E1 = E0*A1;
C1 = E1*X;

% Rigid-body motion 2 of the end-effector.
% This is known from forward kinematics of the robot.
A2 = randomH44( );

% Next configuration after the 1st one under rigid-body motion 2.
E2 = E1*A2;
C2 = E2*X;

% Next configuration after the 1st one under rigid-body motion 2.
B1 = inv(C0)*C1;
B2 = inv(C1)*C2;

% Compute hand-eye transformation
A = [A1,A2];
B = [B1,B2];

X_calibration = handeyecalib( A, B );

% Results
disp('Calibration result:'); X_calibration,
disp('Ground truth:'); X,
