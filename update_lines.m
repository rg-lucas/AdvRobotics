function [V1_updated, V2_updated] = update_lines(T1, V1, T2, V2, M_star)
    % Normalize vectors
    V1 = V1 / norm(V1);
    V2 = V2 / norm(V2);
    % Configuration settings
    max_iterations = 1000;
    tolerance = 1e-5;
    step_size = 0.1;
    % Iterations to adjust the distance
    for iter = 1:max_iterations
        % Update V1 to point towards M*
        V1_updated = M_star - T1;
        V1_updated = V1_updated / norm(V1_updated);
        % Update V2 to maintain the distance between the lines close to zero
        V2_updated = M_star - T2;
        V2_updated = V2_updated - dot(V2_updated, V1_updated) * V1_updated;
        V2_updated = V2_updated / norm(V2_updated);
        % Check the distance between the lines
        current_distance = distance_between_two_lines(T1, V1_updated, T2, V2_updated);
        % Check for convergence
        if abs(current_distance) < tolerance
            break;
        end
        % Update V1 towards M*
        V1_updated = V1_updated + step_size * (M_star - T1);
        V1_updated = V1_updated / norm(V1_updated);
    end
    % Display the final distance between the lines
    disp(['Final distance between the lines: ' num2str(current_distance)]);
    % Check if the maximum number of iterations is reached
    if iter == max_iterations
        warning('Maximum number of iterations reached. The desired distance may not have been completely achieved.');
    end
end

