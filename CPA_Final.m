%%% Lucas And Gustavo - HW6 - CPA
clear
close all
clc

% Define M*
M_star = [2; 2; 1];
% Position of Camera 1
T1 = [0; 0; 0];
% Camera 1 Reference Vector (pointing to the global Z axis)
V1 = [1; 2; 3];
% Camera position 2
% Rotation of 30 degrees in relation to the global Z axis
theta = deg2rad(30);
rotation_matrix = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
T2 = rotation_matrix * [3; 0; 0.25];
% Camera 2 Reference Vector (pointing to the global Z axis)
V2 = -[3; 1; -1];
% Initial chart settings
figure;
hold on;
grid on;
axis equal;
% Plot of Camera 1 and Reference Vector V1
plot3([T1(1), T1(1) + V1(1)], [T1(2), T1(2) + V1(2)], [T1(3), T1(3) + V1(3)], 'Color', 'b', 'LineWidth', 2);
scatter3(T1(1), T1(2), T1(3), 100, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
% Plot of Camera 2 and Reference Vector V2
plot3([T2(1), T2(1) + V2(1)], [T2(2), T2(2) + V2(2)], [T2(3), T2(3) + V2(3)], 'Color', 'r', 'LineWidth', 2);
scatter3(T2(1), T2(2), T2(3), 100, 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
% Plot of point M*
scatter3(M_star(1), M_star(2), M_star(3), 100, 'g', 'filled');
% labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Camera Positions and Reference Vectors');
legend( 'V1', 'Câmera 1',  'V2', 'Câmera 2', 'Ponto M*');
hold off;

% update V1 and V2
[V1_updated, V2_updated] = update_lines(T1, V1, T2, V2, M_star);

disp('Updated Camera 1 Reference Vector (V1_updated):');
disp(V1_updated);
disp('Camera 2 Reference Vector (V2):');
disp(V2_updated);
figure;
hold on;
grid on;
axis equal;
scale_factor = 5; % Ajuste o fator conforme necessário
% Plot Camera 1 and Reference Vector V1
plot3([T1(1), T1(1) + scale_factor * V1_updated(1)], ...
      [T1(2), T1(2) + scale_factor * V1_updated(2)], ...
      [T1(3), T1(3) + scale_factor * V1_updated(3)], 'Color', 'b', 'LineWidth', 2);
scatter3(T1(1), T1(2), T1(3), 100, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
scale_factor = 5; % Ajuste o fator conforme necessário
% Plot of Camera 2 and Reference Vector V2_updated
plot3([T2(1), T2(1) + V2_updated(1)], [T2(2), T2(2) + V2_updated(2)], [T2(3), T2(3) + V2_updated(3)], 'Color', 'r', 'LineWidth', 2);
scatter3(T2(1), T2(2), T2(3), 100, 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
% Plot point M*
scatter3(M_star(1), M_star(2), M_star(3), 100, 'g', 'filled');
%labels e titles
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Updated Camera Positions and Reference Vectors');
legend( 'V1_updated', 'Câmera 1',  'V2_updated', 'Câmera 2', 'Point M*');
hold off;

%Distance between lines and between lines and point M*
distancia_entre_retas = distance_between_two_lines(T1, V1_updated, T2, V2_updated);
disp(['The distance between the lines V1_updated nd V2_updated is: ' num2str(distancia_entre_retas)]);
distancia_V1_M_star = distance_of_guess_point(T1, V1_updated, M_star);
disp(['The distance from V1_updated to M* is: ' num2str(distancia_V1_M_star)]);




% Calling the function to update V2 to cross M*
V2_final = update_V2_to_cross_M_star(T2, V2_updated, M_star);
figure;
hold on;
grid on;
axis equal;
scale_factor = 5;
% Plot of Camera 1 and Reference Vector V1_updated
plot3([T1(1), T1(1) + scale_factor * V1_updated(1)], ...
      [T1(2), T1(2) + scale_factor * V1_updated(2)], ...
      [T1(3), T1(3) + scale_factor * V1_updated(3)], 'Color', 'b', 'LineWidth', 2);
scatter3(T1(1), T1(2), T1(3), 100, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
scale_factor = 5;
% Plot of Camera 2 and Reference Vector V2_final
plot3([T2(1), T2(1) + V2_final(1)], [T2(2), T2(2) + V2_final(2)], [T2(3), T2(3) + V2_final(3)], 'Color', 'r', 'LineWidth', 2);
scatter3(T2(1), T2(2), T2(3), 100, 'o', 'MarkerEdgeColor', 'm', 'MarkerFaceColor', 'r');
% Plot point M*
scatter3(M_star(1), M_star(2), M_star(3), 100, 'g', 'filled');
% Labels e title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Camera Positions and Final Reference Vectors');
legend( 'V1_updated', 'Câmera 1',  'V2_final', 'Câmera 2', 'Ponto M*');
hold off;
% Distance between the lines after the final update
distancia_entre_retas_final = distance_between_two_lines(T1, V1_updated, T2, V2_final);
disp(['A distância final entre as retas é: ' num2str(distancia_entre_retas_final)]);


figure;
hold on;
grid on;
axis equal;
% Plot of Camera 1 and Reference Vector V1
h1 = plot3([T1(1), T1(1) + V1(1)], [T1(2), T1(2) + V1(2)], [T1(3), T1(3) + V1(3)], 'Color', 'b', 'LineWidth', 2);
scatter3(T1(1), T1(2), T1(3), 100, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
% Camera 2 Plot and V2 Reference Vector
h2 = plot3([T2(1), T2(1) + V2(1)], [T2(2), T2(2) + V2(2)], [T2(3), T2(3) + V2(3)], 'Color', 'r', 'LineWidth', 2);
scatter3(T2(1), T2(2), T2(3), 100, 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
% Plot point M*
h3 = scatter3(M_star(1), M_star(2), M_star(3), 100, 'g', 'filled');
% labels e titles
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Animation: Convergence of V1 to M*');
disp('Updated Camera 1 Reference Vector (V1_updated):');
disp(V1_updated);
disp('Updated Camera 2 Reference Vector (V2_final):');
disp(V2_final);
%Distance between lines and between lines and point M*
distancia_entre_retas = distance_between_two_lines(T1, V1_updated, T2, V2_final);
disp(['The distance between the lines V1_updated nd V2_final is: ' num2str(distancia_entre_retas)]);
distancia_V1_M_star = distance_of_guess_point(T1, V1_updated, M_star);
disp(['The distance from V2_final to M* is: ' num2str(distancia_V1_M_star)]);
% Initial settings for the animation
num_frames = 500;
% Animation: Convergence of V1 to M*
for frame = 1:num_frames
    % Update V1 to point to M*
    V1_updated = (1 - frame/num_frames) * V1 + (frame/num_frames) * (M_star - T1);
    V1_updated = V1_updated / norm(V1_updated);
    % Update the plot
    set(h1, 'XData', [T1(1), T1(1) + V1_updated(1)], 'YData', [T1(2), T1(2) + V1_updated(2)], 'ZData', [T1(3), T1(3) + V1_updated(3)]);
    % Pause for visualization
    pause(0.01);
end

% Animation: Adjustment of V2 to the new V1
for frame = 1:num_frames
    % Update V2 to maintain the distance between the lines close to zero
    V2_updated = V2 + dot((V1_updated - V2), V1_updated) / dot(V1_updated, V1_updated) * V1_updated;
    V2_updated = V2_updated / norm(V2_updated);
    % Update the plot
    set(h2, 'XData', [T2(1), T2(1) + V2_updated(1)], 'YData', [T2(2), T2(2) + V2_updated(2)], 'ZData', [T2(3), T2(3) + V2_updated(3)]);
    % Pause for visualization
    pause(0.01);
end
% Animation: Final adjustment of V2 to cross M*
for frame = 1:num_frames
    % Update V2 to cross M* without modifying V1_updated
    V2_final = cross(V1_updated, M_star - T2);
    V2_final = V2_final / norm(V2_final);
    % Update the plot
    set(h2, 'XData', [T2(1), T2(1) + V2_final(1)], 'YData', [T2(2), T2(2) + V2_final(2)], 'ZData', [T2(3), T2(3) + V2_final(3)]);
    % Pause for visualization
    pause(0.01);
end



