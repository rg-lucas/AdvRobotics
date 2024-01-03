A = [1,7];
B = [6,1];
N = 5; % You can change this number for number of points
x = [A(1) B(1)];
y = [A(2) B(2)];
scatter(x,y,'filled')
hold on
New_x = linspace(x(1),x(2),N+2); % Includes given points
New_y = linspace(y(1),y(2),N+2); % Includes given points
New_x(end)=[]
New_x(1)=[]
New_y(end)=[]
New_y(1)=[]
scatter(New_x,New_y,'r','filled')
