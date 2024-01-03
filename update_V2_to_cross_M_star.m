function V2_final = update_V2_to_cross_M_star(T2, V2_updated, M_star)
    % Normalize the vector
    V2_updated = V2_updated / norm(V2_updated);

    % Calculate the direction of V2_updated to pass through M*
    V2_direction = M_star - T2;

    % Project V2_direction onto the direction perpendicular to V2_updated
    projection = V2_direction - dot(V2_direction, V2_updated) * V2_updated;

    % Update V2_updated to pass through M*
    V2_final = V2_updated + projection;
    V2_final = V2_final / norm(V2_final);
end

