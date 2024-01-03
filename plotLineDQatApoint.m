function plotLineDQatApoint (ell_hat, pointDQ, len, c)
  % len : length of the line for plotting
  % c : color of the line, e.g., for black, use 'k'
  %pointDQ : point at which the line is visualized

  L = [ ell_hat(2:4); ell_hat(6:8) ];
  q = pointDQ(6:8);
  plotPluckerLineAtaPoint( L, q, len, c);

end

