% rotation about the origin
clear; close all;
p = [0;5]; %point in the cartesian coordinates
theta = linspace(0,2*pi);
traj = []; %to keep in memory the trajectory
n = 10; % length of the previous trajectory
figure; % open a figure to use in the animation
r = sqrt(p(1)^2+p(2)^2); %gets radius

for i=1:length(theta)
  q = rot2(theta(i))*p; % rotate the point abaout the origin
  clf; % clear the figure
  plot(q(1),q(2), 'bo', 'markersize', 10, 'markerfacecolor', 'b');
  %axis([-2 2 -2 2]); grid on; hold on;
  axis([-(r+1), (r+1), -(r+1), (r+1)]); daspect([1,1,1]); grid on; hold on;

  %show the recent trajectory of the moving point
  traj = [q, traj]; % stack the data: last in first out
  if (i < n) traj_length = i; else traj_length = n; endif
  plot(traj(1,1:traj_length), traj(2,1:traj_length), 'b.');

  pause(0.1); % waits 0.1 seconds
  drawnow; % draw everything now
endfor
