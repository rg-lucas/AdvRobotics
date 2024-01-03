function reconstructed_points = reconstruct_3d(correspondences, K)
    % Input:
    %   correspondences: Nx4 matrix, each row represents a correspondence
    %                    [u1, v1, u2, v2], where (u1, v1) and (u2, v2) are
    %                    corresponding points in image 1 and image 2.
    %   K: Camera calibration matrix (3x3)

    % Extract correspondences
    points_image1 = correspondences(:, 1:2);
    points_image2 = correspondences(:, 3:4);

    % Normalize image coordinates
    normalized_points1 = inv(K) * [points_image1, ones(size(points_image1, 1), 1)]';
    normalized_points2 = inv(K) * [points_image2, ones(size(points_image2, 1), 1)]';

    % Linear triangulation
    reconstructed_points_homogeneous = linear_triangulation(normalized_points1, normalized_points2);

    % Bundle Adjustment (assuming a simplified case, not a full bundle adjustment)
    % You might want to use a dedicated library for a more robust implementation
    reconstructed_points = bundle_adjustment(correspondences, K, reconstructed_points_homogeneous);
end

function reconstructed_points_homogeneous = linear_triangulation(points1, points2)
    % Linear triangulation based on homogeneous coordinates

    A = [
        points1(1)*points2(3) - points1(3)*points2(1), points1(1)*points2(6) - points1(4)*points2(1), 0;
        points1(2)*points2(3) - points1(3)*points2(2), points1(2)*points2(6) - points1(5)*points2(2), 0;
    ];

    [~, ~, V] = svd(A);

    reconstructed_points_homogeneous = V(:, end);
end



function reconstructed_points = bundle_adjustment(correspondences, K, reconstructed_points_homogeneous)
    % Simple bundle adjustment (assuming a fixed camera calibration matrix)

    % Extract correspondences
    points_image1 = correspondences(:, 1:2);
    points_image2 = correspondences(:, 3:4);

    % Initialize 3D points
    reconstructed_points = reconstructed_points_homogeneous(1:3, :)';

    % Iterate for a few steps (you might need a more sophisticated approach)
    for iter = 1:5
        % Project 3D points to image planes
        projected_points_image1 = K * [eye(3), zeros(3, 1)] * [reconstructed_points, ones(size(reconstructed_points, 1), 1)]';
        projected_points_image2 = K * [eye(3), zeros(3, 1)] * [reconstructed_points, ones(size(reconstructed_points, 1), 1)]';

        % Reprojection error
        error_image1 = points_image1 - projected_points_image1(1:2, :)';
        error_image2 = points_image2 - projected_points_image2(1:2, :)';

        % Update 3D points
        delta_points = [pinv(K) * [error_image1, ones(size(error_image1, 1), 1)]'; pinv(K) * [error_image2, ones(size(error_image2, 1), 1)]'];
        delta_points = delta_points(1:3, :)';
        reconstructed_points = reconstructed_points + delta_points;

        % Display the reprojection error (optional)
        disp(['Iteration ', num2str(iter), ', Mean Reprojection Error: ', num2str(mean([norm(error_image1, 'fro'), norm(error_image2, 'fro')])), ' pixels']);
    end
end

