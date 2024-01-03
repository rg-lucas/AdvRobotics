function plot_curve(curve)
% Plot the curve
plot3(curve(1,:), curve(2,:), curve(3,:), '-o');
axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
title('Curve with radius R');
end

