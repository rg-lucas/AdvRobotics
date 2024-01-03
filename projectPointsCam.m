
function [mi_rf, mi_ss] = projectPointsCam(H, K, f, M)
  % object in camera's POV
  R = H(1:3,1:3);
  t = H(1:3,4);
  Rt = [R', -R'*t];
  Z_mi = K * Rt * [M; ones(1, size(M, 2))];
  mi_ss = round(Z_mi ./ Z_mi(3,:));

  % Formulation using matrix-vector representation
  % Transformation from image plane - sensor space coordinates to 3D rays of points

  q = inv(K) * mi_ss;
  mi_camf = f * (q(1:3,:) ./ q(3,:));
  mi_rf = H * [mi_camf; ones(1, size(mi_camf, 2))];
  mi_rf = mi_rf(1:3,:) ./ mi_rf(4,:);
end

