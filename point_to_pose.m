function pose = point_to_pose(point)
  % Create a 4x4 identity matrix
  pose = eye(4);

  % Set the translation portion of the pose matrix
  pose(1:3,4) = point';

endfunction
