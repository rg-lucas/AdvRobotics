function [m_cam1, m_cam2] = triangulation_and_projection(M_world, R1, T1, R2, T2, K1, K2)
    % Express M in Camera Coordinate Frames
    M_cam1 = R1 * (M_world - T1);
    M_cam2 = R2 * (M_world - T2);

    % Homogeneous coordinates
    M_homog_cam1 = [M_cam1];
    M_homog_cam2 = [M_cam2];

    % Project M onto the Rays
    m_homog_cam1 = K1 * M_homog_cam1;
    m_homog_cam2 = K2 * M_homog_cam2;

    % Normalize homogeneous coordinates
    m_cam1 = m_homog_cam1 / m_homog_cam1(3);
    m_cam2 = m_homog_cam2 / m_homog_cam2(3);
end

