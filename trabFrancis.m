close; clear; clc; % Clearing everything

% Defining the first line
phi_1 = deg2rad(60);
d_1 = 1;

x_min_1 = -4;
x_max_1 = 4;

y_min_1 = (d_1-x_min_1*cos(phi_1))/sin(phi_1);
y_max_1 = (d_1-x_max_1*cos(phi_1))/sin(phi_1);

%homo_coord_1 = [cos(phi_1); sin(phi_1); -d_1] % Retruns the vector with the homogenous coordinates

% Plotting the first line
plot([x_min_1 x_max_1], [y_min_1 y_max_1], 'k', 'linewidth', 2);
hold on;

% Defining the second line
phi_2 = deg2rad(15);
d_2 = -1;

x_min_2 = -4;
x_max_2 = 4;

y_min_2 = (d_2-x_min_2*cos(phi_2))/sin(phi_2);
y_max_2 = (d_2-x_max_2*cos(phi_2))/sin(phi_2);

%homo_coord_2 = [cos(phi_2); sin(phi_2); -d_2] % Retruns the vector with the homogenous coordinates

% Plotting the second line
plot([x_min_2 x_max_2], [y_min_2 y_max_2], 'k', 'linewidth', 2);

% Calculating and plotting the intersection
y_int = (d_2*cos(phi_1)-d_1*cos(phi_2))/(sin(phi_2)*cos(phi_1)-sin(phi_1)*cos(phi_2));
x_int = (d_1-y_int*sin(phi_1))/cos(phi_1);
plot(x_int, y_int, 'r.', 'markersize', 20);
