function plotCam3d( H, K, f )
  % H: 4x4 pose matrix
  % K: camera intrinsic matrix 3x3
  % f: camera focal length in meters (along z-axis)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Extract parameters from camera intrinsic matrix

  % pixel dimensions
  sx = f/K(1,1); % pixel size along x-axis (meters)
  sy = f/K(2,2); % pixel size along y-axis (meters)

  %% projection coordinates in the image plane - metric space
  %% principal point (image center)
  pu = K(1,3)*sx; % (along x-axis)
  pv = K(2,3)*sy; % (along y-axis)

  %% image size metric space
  Width_m = pu*2; % meters (along x-axis)
  Height_m = pv*2; % meters (along y-axis)

  %% image size sensor space
  Width_pix = Width_m/sx; % pixels (along x-axis)
  Height_pix = Height_m/sy; % pixels (along y-axis)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% visualize the image plane borders in the reference frame
  upperleftcorner = [ -Width_m/2; -Height_m/2; f ];
  upperrightcorner = [ Width_m/2; -Height_m/2; f ];
  lowerrightcorner = [ Width_m/2; Height_m/2; f ];
  lowerleftcorner = [ -Width_m/2; Height_m/2; f ];

  image_bordersCam = [ upperleftcorner, upperrightcorner, lowerrightcorner, lowerleftcorner, upperleftcorner ];

  % homogeneous transformation of image borders of the camera coordinate frame to the reference frame
  image_borders =  H*[ image_bordersCam;  ones(1,5) ];
  image_borders = image_borders./ image_borders(4,:);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Plot
  plotCoordinateFramefromPose( H, 0.06 ); hold on; % from Basic Robotics
  axis equal; title('Camera coordinate frame - 3D metric space','fontsize',20)
  plot3( image_borders(1,:), image_borders(2,:), image_borders(3,:), 'b', 'linewidth', 4 );

end
