close all;clear;clc;
% cartesian coordinates of a 2D point
p = [3;2;1];
xlabel("X");
ylabel("Y");
grid on;
pause();

% homogeneous coordinates of a 2D poiont with different lambda value
hold on;
%for lambda = 0:2:2
P = [lambda*p];
plot(P(1),P(2), 'b.', 'markersize', 30, 'linewidth',4);
grid on;

%end
%pause();
%
lambda = 0:2:2;
P = [lambda*p; lambda];
plot(P(1,:),P(2,:), 'k', 'linewidth', 4);
