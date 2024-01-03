clear; close all; clc;

%% create 2 points and check if they are in the range of the robot  or not.

A = 1.4*rand(1,2)
B = 1.4*rand(1,2)

C = sqrt(A(1)^2+A(2)^2)
D = sqrt(B(1)^2+B(2)^2)

%% Checking if A and B are in the range of the robot
if C < 1.5 && D < 1.5
  printf('the range is  valid')
else
  printf('the range is not valid')
end

clf; hold on;
 plot( A(2),A(1), 'b', 'markersize', 20);
 plot( B(2),B(1), 'r', 'markersize', 20);
 axis equal; axis([-2.5 2.5 0 2.5]);
 xlabel('X', 'fontsize', 20); ylabel('Y', 'fontsize', 20); grid on;
drawnow;
