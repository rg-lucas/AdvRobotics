function plot2lines( homogeneous_coordinatesl, homogeneous_coordinatesm, xaxis_interval )

    %Calculate l parameters
    cos_phil = homogeneous_coordinatesl(1);
    sin_phil = homogeneous_coordinatesl(2);
    dl = -homogeneous_coordinatesl(3);

    x_minl = xaxis_interval(1);
    x_maxl = xaxis_interval(2);

    y_min_sin_phil = dl - x_minl*cos_phil;
    y_minl = y_min_sin_phil / sin_phil;

    y_max_sin_phil = dl - x_maxl*cos_phil;
    y_maxl = y_max_sin_phil / sin_phil;

    %calculate m parameters
    cos_phim = homogeneous_coordinatesm(1);
    sin_phim = homogeneous_coordinatesm(2);
    dm = -homogeneous_coordinatesm(3);

    x_minm = xaxis_interval(1);
    x_maxm = xaxis_interval(2);

    y_min_sin_phim = dm - x_minm*cos_phim;
    y_minm = y_min_sin_phim / sin_phim;

    y_max_sin_phim = dm - x_maxm*cos_phim;
    y_maxm = y_max_sin_phim / sin_phim;





    plot( [[x_minl, x_maxl], [x_minm, x_maxm]], [[y_minl, y_maxl], [y_minm, y_maxm]], 'k', 'linewidth', 4);

    endfunction
