function plotPluckerLineAtaPoint( L, q, len, c )

u = L(1:3);
p1 = q + 0.5*len*u;
p2 = q - 0.5*len*u;

plot3( [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'linewidth', 4, 'color', c );

end
