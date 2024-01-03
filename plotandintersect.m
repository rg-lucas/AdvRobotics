function plotandintersect( l, m , xaxis_interval )

    %Plots line l
    phi_l = deg2rad(l(1))
    %cos_phi_l = cos(l(1));
    %sin_phi_l = sin(l(1));
    d_l = l(2);

    x_min = xaxis_interval(1);
    x_max = xaxis_interval(2);

    y_min_l = d_l - x_min*cos(phi_l) / sin(phi_l);
    y_max_l = d_l - x_max*cos(phi_l) / sin(phi_l);

    plot( [x_min, x_max], [y_min_l, y_max_l], 'k', 'linewidth', 2);
    hold on; grid on;

    %Plots line l
    phi_m = deg2rad(m(1))

    %%cos_phi_m = cos(m(1));
    %%sin_phi_m = sin(m(1));
    d_m = m(2);

    y_min_m = d_m - x_min*cos(phi_m) / sin(phi_m);
    y_max_m = d_m - x_max*cos(phi_m) / sin(phi_m);

    plot( [x_min, x_max], [y_min_m, y_max_m], 'k', 'linewidth', 2);
    hold on; grid on;

    % Calculating and plotting the intersection
    y_int = (d_m*cos(phi_l)-d_l*cos(phi_m))/(sin(phi_m)*cos(phi_l)-sin(phi_l)*cos(phi_m));
    x_int = (d_l-y_int*sin(phi_l))/cos(phi_l);
    plot(x_int, y_int, 'r.', 'markersize', 20);




    endfunction
