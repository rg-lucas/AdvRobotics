% plots the plucker line with a desired length and color
function plotPluckerLineV2( L, a, b, c )
% L : plucker line coordinates
% len : length of the line for plotting
% c : color of the line, e.g., for black, use 'k'

eL = EucNormalizationPluckerLine( L );
eLh = eL(1:3);
eLo = eL(4:6);

% points of the line

p1 = a;
p2 = b;

plot3( [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'linewidth', 4, 'color', c );
grid on;

end
