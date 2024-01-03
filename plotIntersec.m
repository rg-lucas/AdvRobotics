function plotIntersec( l, m , interval )

close;

% Defines first line
phi_1 = deg2rad(l(1));
d_1 = l(2);

x_min = interval(1);
x_max = interval(2);

y_min_1 = (d_1-x_min*cos(phi_1))/sin(phi_1);
y_max_1 = (d_1-x_max*cos(phi_1))/sin(phi_1);


% Plots the first line
plot([x_min x_max], [y_min_1 y_max_1], 'k', 'linewidth', 2);
hold on;

% Defines second line
phi_2 = deg2rad(m(1));
d_2 = m(2);


y_min_2 = (d_2-x_min*cos(phi_2))/sin(phi_2);
y_max_2 = (d_2-x_max*cos(phi_2))/sin(phi_2);


% Plots second line
plot([x_min x_max], [y_min_2 y_max_2], 'k', 'linewidth', 2);

% Defines and plots intersection
y_int = (d_2*cos(phi_1)-d_1*cos(phi_2))/(sin(phi_2)*cos(phi_1)-sin(phi_1)*cos(phi_2));
x_int = (d_1-y_int*sin(phi_1))/cos(phi_1);
plot(x_int, y_int, 'r.', 'markersize', 20);

    endfunction
