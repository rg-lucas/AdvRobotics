function visualize_scene(m_cam1, m_cam2, P1, P2, M_world, R1, T1, R2, T2)
    % Create a figure
    figure;
    % Plot 3D point
    scatter3(M_world(1), M_world(2), M_world(3), 10, 'ro', 'filled', 'LineWidth', 2);
    hold on;
    % Plot cameras
    plot3(T1(1), T1(2), T1(3), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'LineWidth', 2); % Camera 1
    plot3(T2(1), T2(2), T2(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2); % Camera 2
    % Plot rays (animation)
    num_frames = 30; % Number of frames for the animation
    frames = linspace(0, 1, num_frames);
    for t = frames
        % Interpolate camera positions
        interpolated_T1 = (1 - t) * T1;
        interpolated_T2 = t * T2;
        % Interpolate rays
        ray_cam1 = P1' * [R1, T1] * [M_world; 1];
        ray_cam2 = P2' * [R2, T2] * [M_world; 1];
        % Normalize homogeneous coordinates
        ray_cam1 = ray_cam1 ./ ray_cam1(4);
        ray_cam2 = ray_cam2 ./ ray_cam2(4);
        % Plot rays
        plot3([interpolated_T1(1), ray_cam1(1)], [interpolated_T1(2), ray_cam1(2)], [interpolated_T1(3), ray_cam1(3)], 'b-');
        plot3([interpolated_T2(1), ray_cam2(1)], [interpolated_T2(2), ray_cam2(2)], [interpolated_T2(3), ray_cam2(3)], 'g-');
        % Pause to create animation effect
        pause(0.1);
        % Clear the previous rays
        clf;
        hold on;
        % Re-plot 3D point and cameras
        scatter3(M_world(1), M_world(2), M_world(3), 10, 'ro', 'filled', 'LineWidth', 2);
        plot3(T1(1), T1(2), T1(3), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'LineWidth', 2);
        plot3(T2(1), T2(2), T2(3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    end
    hold off;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Scene Visualization');
end



