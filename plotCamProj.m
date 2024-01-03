function plotCamProj( H, K, f, M, mi_rf )
  % H: 4x4 pose matrix
  % K: camera intrinsic matrix 3x3
  % f: camera focal length in meters (along z-axis)
  % M: an object pointcloud in the reference frame
  % mi_rf : projection coordinates in the camera frame

  plotCam3d(H, K, f);
  plot3( M(1,:), M(2,:), M(3,:), 'ro', 'markersize', 10, 'markerfacecolor', 'r' ); hold on;
  plot3( M(1,:), M(2,:), M(3,:), 'k', 'linewidth', 4 );

  t = H(1:3,4);
  unitrays = (mi_rf-t)./norm(mi_rf-t);
  for i=1:length(unitrays(1,:))
    plot3( [t(1), t(1) + 2*unitrays(1,i)], [t(2), t(2) + 2*unitrays(2,i)], [t(3), t(3) + 2*unitrays(3,i)], 'r', 'linewidth', 1 );
  end
  view(-0.215,0.038);
  % plot the 2nd camera's projection coordinates in the 1st camera coordinate frame
  plot3( mi_rf(1,:), mi_rf(2,:), mi_rf(3,:), 'bo', 'markersize', 10, 'markerfacecolor', 'r' ); hold on;
  plot3( mi_rf(1,:), mi_rf(2,:), mi_rf(3,:), 'k', 'linewidth', 4 );

  end
