clear; close all; clc;

A = (0.4*(rand(3,1)+1) - [-0.288-0.08;  0;      0.0645-.08-.26-.203   ])
B = ([0.4*(rand(1,1)+1); 0.4*(rand(1,1)+1); 0.4*(rand(1,1)+1)] - [-0.288-0.08;  0;      0.0645-.08-.26-.203   ])
B = [-B(1); B(2); -B(3)]
n = 10;

%% Checking if the coordinates x, y aren't outside of the robots reach
%% Using a simplified checking process.

if (sqrt(A(1)^2) < 0.001*854.889-(-0.288-0.08) && sqrt(A(1)^2) > 0.001*241.34-(-0.288-0.08) && sqrt(A(2)^2) < 0.001*854.88 && sqrt(A(2)^2) > 0.001*241.34 &&
  sqrt(B(1)^2) < 0.001*854.88-(-0.288-0.08) && sqrt(B(1)^2) > 0.001*241.34-(-0.288-0.08) && sqrt(B(2)^2) < 0.001*854.88 && sqrt(B(2)^2) > 0.001*241.34 &&
  sqrt(A(3)^2) < 0.001*779.88-(0.0645-.08-.26-.203) && sqrt(A(3)^2) > 0.001*166.34-(0.0645-.08-.26-.203) &&
  sqrt(B(3)^2) < 0.001*779.88-(0.0645-.08-.26-.203) && sqrt(B(3)^2) > 0.001*166.34-(0.0645-.08-.26-.203))

  printf('the range is  valid in X, Y, Z\n')

else
  printf('the range is not valid in X,Y, Z\n')

end



x = [A(1), B(1)];
y = [A(2), B(2)];
z = [A(3), B(3)];

%% Create the interpolations and the values of each point

New_x = linspace(x(1),x(2),n+2); % Includes given points
New_y = linspace(y(1),y(2),n+2); % Includes given points
New_z = linspace(z(1),z(2),n+2); % Includes given points
New_x(end)=[];
New_x(1)=[];
New_y(end)=[];
New_y(1)=[];
New_z(end)=[];
New_z(1)=[];

%% interpolated points for the robot pass through


p_inter = [A(1), New_x, B(1); A(2), New_y, B(2); A(3), New_z, B(3)]


% Plot the graph and the points interpolations

clf; hold on;

 plot3(New_x,New_y,New_z, 'r.', 'markersize', 10);
 axis equal; axis([-1.5 1.5 0 1.5 0 1.5]);
 xlabel('X', 'fontsize', 20); ylabel('Y', 'fontsize', 20); zlabel('Z', 'fontsize', 20); grid on;
drawnow;

