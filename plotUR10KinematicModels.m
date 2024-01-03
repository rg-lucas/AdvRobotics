function plotUR10KinematicModels (theta, ttl)
  n = 7; % number of axis of rotations
  clf;
  plotSize = [-1 1 -1 1 -.2 1]; %size of the graphic plotted
  He0 = eye(4);

  [He, xi_prime, q_prime] = UR10KinematicModels(theta);

  plotRobot(q_prime, xi_prime );
  plotCoordinateFramefromPose( He0, 0.1 );
  set (gca (), 'zdir', 'reverse'); axis equal;
  axis(plotSize); grid on;
  title(ttl,'fontsize',20);
  drawnow; pause(0.1);

