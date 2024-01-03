% Question 1 - Advanced Robotics - Homework 5
% Lucas Resende Gomes

clear;
close all;
clc;

% Desired pose from angle-axis + translation
theta = rand; % angle
u = rand(3);
u = u/norm(u); % axis
t = rand(3,1); % translation;

Hd = [ rodrigues(u, theta), t; zeros(1,3), 1 ];


% Current pose from angle-axis + translation
theta = rand; % angle
u = rand(3);
u = u/norm(u); % axis
t = rand(3,1); % translation;

H = [ rodrigues(u, theta), t; zeros(1,3), 1 ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
eps = 0.001; % small threshold value to stop from error
iter = 0; maxIter = 10000;
traj = zeros(3,maxIter); % this will keep the trajectory

He = inv(H)*Hd;
wSv = logm(He);

while(norm(wSv) > eps )
He = inv(H)*Hd;

thetau = logm(He(1:3,[1:3]));
epsilon = [ thetau(3,[2]); thetau(1,[3]); thetau(2,[1]);
            transpose(He(1:3,[1:3]))*He(1:3,[4])   ];
wSv = twist2homogenousform(epsilon);

lambda = 50; % control gain
dt = 0.001; % time step
H = H*expm(dt*lambda*wSv); % new pose
%% loop counter
iter = iter + 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
traj(:,iter) = H(1:3,4); % keep trajectory
clf; hold on; grid on; s = 0.2;
plotCoordinateFramefromPose( H, s ); % plot current coordinate frame
plotCoordinateFramefromPose( Hd, s ); % plot desired coordinate frame
plot3( traj(1,1:iter), traj(2,1:iter), traj(3,1:iter), 'g', 'linewidth', 2);
title(['twist norm: ',num2str(norm(wSv))],'fontsize',20);
axis equal; view( -45, 30 ); axis([-0.5 1.5 -0.5 1.5 -0.5 1.5]);
drawnow;
end
%until( norm(wSv) < eps ) % break when twist amlitude is smaller than epsilon
