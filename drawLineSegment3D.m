function drawLineSegment3D( head, tail)

  plot3( [ tail(1), head(1)], [ tail(2), head(2)], [ tail(3), head(3)], 'k', 'linewidth', 10); hold on;
  plot3( tail(1), tail(2), tail(3), 'ro', 'markersize', 20, 'markerfacecolor', 'r' );
  plot3( head(1), head(2), head(3), 'bo', 'markersize', 20, 'markerfacecolor', 'r' );

end

