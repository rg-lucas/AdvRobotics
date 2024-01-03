function plot2Dline( normalized_line, xaxis_interval, color )

    cos_phi = normalized_line(1);
    sin_phi = normalized_line(2);
    d = -normalized_line(3);

    x_min = xaxis_interval(1);
    x_max = xaxis_interval(2);

    y_min_sin_phi = d - x_min*cos_phi;
    y_min = y_min_sin_phi / sin_phi;
    y_max_sin_phi = d - x_max*cos_phi;
    y_max = y_max_sin_phi / sin_phi;



    plot( [x_min, x_max], [y_min, y_max], color, 'linewidth', 4);

end
