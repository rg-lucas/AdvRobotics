close all; clear; clc;

p=[3;2];
%plot (p(1), p(2), 'r.', 'markersize', 20, 'linewidth', 4);
xlabel("X");
ylabel("Y");
grid on;
pause();

hold on;
for lambda = 0:0.1:2
  P = [lambda.*p; lambda];
  plot3(P(1), P(2), P(3), 'b.', 'markersize', 30, 'linewidth', 4);
  grid on;
  zlabel("Lambda")
end;
pause();

lambda = 0:0.1:2;
P = [lambda.*p; lambda];
plot3(P(1,:), P(2,:), P(3,:), 'k', 'linewidth', 4);
