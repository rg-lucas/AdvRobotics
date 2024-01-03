function plotRobotDQ( q, xi )
  % q : joints' locations in Dual Quaternions
  % xi : joints' screw axes in Dual Quaternions

  plot3( q(6,:), q(7,:), q(8,:), 'LineWidth', 4, 'Color', 'k');
  hold on;

  for i=1:size(q,2)
    plot3( q(6,i), q(7,i), q(8,i), 'ro', 'markersize', 10, 'markerfacecolor', 'r');
  end

  plot3( q(6,1), q(7,1), q(8,1), 'bo', 'markersize', 10, 'markerfacecolor', 'b');
  plot3( q(6,end), q(7,end), q(8,end), 'go', 'markersize', 10, 'markerfacecolor', 'g');

  for k=1:size(xi,2)
    plotLineDQatApoint( xi(:,k), q(:,k+1), 0.12, 'r' );
  end

end

