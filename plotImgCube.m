function plotImgCube (H, K, f, mi_ss)

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
  image_borders_ms = image_bordersCam(1:2,:) + [pu;pv]; % Metric Space
  image_borders_ss = [image_borders_ms(1,:)/sx; image_borders_ms(2,:)/sy]; %S ensor Space

  %% Plot points
  plot( mi_ss(1,:),  mi_ss(2,:), 'ro', 'markersize', 10, 'markerfacecolor', 'r' ); hold on;
  plot( mi_ss(1,:),  mi_ss(2,:), 'k', 'linewidth', 4 );
  xlabel(num2str(Width_pix),'fontsize',20);
  ylabel(num2str(Height_pix),'fontsize',20);
  axis ij; % makes the upper left corner the origin

  %% Plot borders
  plot( image_borders_ss(1,:),  image_borders_ss(2,:), 'b', 'linewidth', 6 );
  xlim([0 Width_pix]); ylim([0 Height_pix]);
  xlabel(num2str(Width_pix),'fontsize',20);
  ylabel(num2str(Height_pix),'fontsize',20);
  grid on; title('Camera image - 2D sensor space','fontsize',20);

  end
