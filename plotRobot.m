function plotRobot( q, xi )
  % q : joints' locations
  % xi : joints' screw axes

  plot3( q(1,:), q(2,:), q(3,:), 'LineWidth', 4, 'Color', 'k');
  hold on;

  for i=1:size(q,2)
    plot3( q(1,i), q(2,i), q(3,i), 'ro', 'markersize', 10, 'markerfacecolor', 'r');
  end

  plot3( q(1,1), q(2,1), q(3,1), 'bo', 'markersize', 10, 'markerfacecolor', 'b');
  plot3( q(1,end), q(2,end), q(3,end), 'go', 'markersize', 10, 'markerfacecolor', 'g');

  for k=1:size(xi,2)
    plotPluckerLineAtaPoint( xi(:,k), q(:,k+1), 0.12, 'r' );
  end

end

