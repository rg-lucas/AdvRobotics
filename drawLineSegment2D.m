function drawLineSegment2D( head, tail)

  plot( [tail(1), head(1)], [tail(2), head(2)], 'k', 'linewidth', 10); hold on;
  plot( tail(1), tail(2), 'ro', 'markersize', 20, 'markerfacecolor', 'r');
  plot( head(1), head(2), 'bo', 'markersize', 20, 'markerfacecolor', 'b');

end

