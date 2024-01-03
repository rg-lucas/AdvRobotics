function distance = distance_between_two_lines(P1, V1, P2, V2)
    % Normalized direction vectors
    V1 = V1 / norm(V1);
    V2 = V2 / norm(V2);

    % Vector between the points of the lines
    delta_P = P2 - P1;

    % Vector normal to both lines
    n = cross(V1, V2);

    % Distance between the lines
    distance = abs(dot(delta_P, n)) / norm(n);
end

