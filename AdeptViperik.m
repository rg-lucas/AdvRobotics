function theta = AdeptViperik(Hd, theta)
%% INITIALIZATION
% desired joint configuration to discover

% figure;

% an initial guess which should be close to solution
theta_0 = theta;
%%figure;
%%plotAdeptViperKinematicModels (theta_0, 'Initial guess Pose Adept Viper');
%%plotCoordinateFramefromPose (Hd, 0.1);

eps = 10^(-5); % meters/second
xi=1;
%LOOP
while( norm(xi) > eps )
  %currentend-effector pose of the virtual robot
  [H,J] = AdeptViperKinematicModels ( theta );
  He = inv(H)*Hd; %pose error
  xi_hat = logm(He); % twist in its homogeneous form 4x4 from pose error
  xi = vex3D (xi_hat); % extract twist vector 6x1 from its homogeneus form

  lambda = 0.005; % damping factor for the pseudo-inverse jacobian

  % option 2 : (faster solving without the inverse fuction)
  theta = theta + J'*((J*J' + lambda*eye(6))\xi);

  disp(['norm(xi) ',num2str(norm(xi))]);

end
% END OF LOOP
##figure;
##disp('initial and final joint values');
##[ rad2deg(theta_0), rad2deg(theta) ],
##disp('Final pose error');
##He,
##plotAdeptViperKinematicModels (theta, 'Final pose AdeptViper');
##plotCoordinateFramefromPose (H, 0.1);

end
