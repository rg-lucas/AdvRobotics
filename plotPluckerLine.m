% plots the plucker line with a desired length and color
function plotPluckerLine( L, len, c )
% L : plucker line coordinates
% len : length of the line for plotting
% c : color of the line, e.g., for black, use 'k'

eL = EucNormalizationPluckerLine( L );
eLh = eL(1:3);
eLo = eL(4:6);

% closest point on the line
p = cross( eLh, eLo);

p1 = p + 0.5*len*eLh;
p2 = p - 0.5*len*eLh;

plot3( [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'linewidth', 4, 'color', c );
grid on;

end