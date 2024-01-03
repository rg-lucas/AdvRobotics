function distance = distance_of_point_to_line(T1, V1, point)
    % Vector from T1 to the point
    vec_T1_point = point - T1;

    % Projection of vec_T1_point onto the direction of V1
    projection_V1 = dot(vec_T1_point, V1) / norm(V1)^2 * V1;

    % Vector perpendicular to V1
    perpendicular_V1 = vec_T1_point - projection_V1;

    % Distance from the line (defined by T1 and V1) to the point
    distance = norm(perpendicular_V1);
end

