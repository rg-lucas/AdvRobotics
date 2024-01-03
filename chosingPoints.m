%% create 2 points and check if they are in the range of the robot  or not.

A = 1.4*rand(1,2);
B = [-1.3*rand(1,1), 1.3*rand(1,1)];

%% Check the distance of those points to the base of the robot
C = sqrt(A(1)^2+A(2)^2);
D = sqrt(B(1)^2+B(2)^2);

robotSize = 1.5;


%% Checking if A and B are in the range of the robot
if C < robotSize && D < robotSize
  printf('the range is  valid')

else
  printf('the range is not valid')

end

%% Give the size of the robot to check if the distance is reachable

x = [A(1), B(1)];
y = [A(2), B(2)];
n = 10;

%% Create the interpolations and the values of each point

New_x = linspace(x(1),x(2),n+2); % Includes given points
New_y = linspace(y(1),y(2),n+2); % Includes given points
New_x(end)=[];
New_x(1)=[];
New_y(end)=[];
New_y(1)=[];

%% interpolated points for the robot pass through


p_inter = [A(1), New_x, B(1); A(2), New_y, B(2)];


% Plot the graph and the points interpolations

clf; hold on;
 plot( A(1),A(2), 'b', 'markersize', 20);
 plot( B(1),B(2), 'g', 'markersize', 20);
 plot(x,y);
 scatter(New_x,New_y,'r','filled');
 axis equal; axis([-2.5 2.5 0 2.5]);
 xlabel('X', 'fontsize', 20); ylabel('Y', 'fontsize', 20); grid on;
drawnow;

