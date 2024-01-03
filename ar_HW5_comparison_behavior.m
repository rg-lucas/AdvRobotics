% Lucas Gomes - Question 1 - Advanced Robotics - Homework 5

clear; close all; clc;

% Current pose from angle-axis + translation
theta = rand*2*pi; % angle
u = rand(3);
u = u/norm(u); % axis
t = rand(3,1)*3; % translation;
H1 = [ rodrigues(u, theta), t; zeros(1,3), 1 ];
H2 = H1;

%Desired pose from angle-axis + translation
theta = rand*2*pi; % angle
u = rand(3);
u = u/norm(u); % axis
t = rand(3,1)*3; % translation;
Hd = [ rodrigues(u, theta), t; zeros(1,3), 1 ];


% Dimensions for plotting:
xySize = [min(min([Hd(1:2,4), H1(1:2,4)]))-1,  max(max([Hd(1:2,4), H1(1:2,4)]))+1];
zSize = [min(min([Hd(3,4), H1(3,4)]))-1,  max(max([Hd(3,4), H1(3,4)]))+1];
pSize = [xySize xySize zSize];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
eps = 0.001; % small threshold value to stop from error
iter = 0; maxIter = 10000;
iter2=iter1=2;
traj = zeros(3,maxIter); % this will keep the trajectory

wSv1 = 1;
wSv2 = wSv1; % declaring wSv to enter the while loop

% Declaring velocities vectors to store data for plotting convergence behavior
vels1 = vels2 = zeros(1,2);
wSvs = zeros(1,2);

while(or((norm(wSv1) > eps),(norm(wSv2) > eps)))

%% Error
He1 = inv(H1)*Hd;
He2 = inv(H2)*Hd;
%% Extract velocity twist in its homogenous form from error
thetau = logm(He2(1:3,[1:3])); % obtaining the theta*u matrix from rodrigues matrix inside the error

epsilon = [ thetau(3,[2]); % theta X ux
            thetau(1,[3]); % theta X uy
            thetau(2,[1]); % theta X uz
            transpose(He2(1:3,[1:3]))*He2(1:3,[4])   ];   % transposed Rodrigues X t

wSv1 = logm(He1);
wSv2 = twist2homogenousform(epsilon);

% Getting magnitude of velocities for plotting convergence behavior:
%1st Control Law:
angVel1 = [wSv1(3,2); wSv1(1,3); wSv1(2,1)]; mAngVel1 = norm((angVel1));
linVel1 = wSv1(1:3,4); mLinVel1 = norm((linVel1));
vels1 = [vels1; [mAngVel1, mLinVel1]];

%2nd Control Law:
angVel2 = [wSv2(3,2); wSv2(1,3); wSv2(2,1)]; mAngVel2 = norm((angVel2));
linVel2 = wSv2(1:3,4); mLinVel2 = norm((linVel2));
vels2 = [vels2; [mAngVel2, mLinVel2]];

wSvs = [wSvs; [norm(wSv1), norm(wSv2)]];

%% Uptade the current pose using each control law
lambda = 100; % control gain
dt = 0.001; % time step
H1 = H1*expm(dt*lambda*wSv1); % new pose
H2 = H2*expm(dt*lambda*wSv2); % new pose

%% loop counter
iter = iter + 1;

if (norm(wSv1) >= eps)
  iter1 = iter1 +1;
end
if (norm(wSv2) >= eps)
  iter2 = iter2 +1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
traj1(:,iter) = H1(1:3,4); % keep trajectory
traj2(:,iter) = H2(1:3,4); % keep trajectory
clf; hold on; grid on; s = 0.2;
plotCoordinateFramefromPose( H1, s ); % plot current coordinate frame
plotCoordinateFramefromPose( Hd, s ); % plot desired coordinate frame
plot3( traj1(1,1:iter), traj1(2,1:iter), traj1(3,1:iter), 'g', 'linewidth', 2);
plot3( traj2(1,1:iter), traj2(2,1:iter), traj2(3,1:iter), 'm', 'linewidth', 2);
title(['twist 1: ',num2str(norm(wSv1)),' || twist 2: ',num2str(norm(wSv2))],'fontsize',20);
axis equal; view( -45, 30 ); axis(pSize);
drawnow;
end


% Plotting Convergence Behavior comparison from Twist magnitude
figure; plot(wSvs(:,1), 'b'); hold on; plot(wSvs(:,2), 'r'); plot(iter1,0, 'b', 'LineWidth',0.001,'MarkerSize',20); plot(iter2,0, 'r', 'LineWidth',0.001,'MarkerSize',20);
legend( ['Twist Magnitude Law 1'; 'Twist Magnitude Law 2'; 'Final Iteration Law 1: ' num2str(iter1); 'Final Iteration Law 2: ' num2str(iter2)]); grid on; grid minor;
title('Convergence Behavior Comparison'); ylabel('Twist Magnitude'); xlabel('# of iterations');



