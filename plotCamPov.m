function plotCamPOV(Homography, IntrinsicMatrix, FocalLength, ImagePointsSensorSpace)
  %% Pixel dimensions
  pixelSizeX = FocalLength / IntrinsicMatrix(1,1); % Pixel size along x-axis (meters)
  pixelSizeY = FocalLength / IntrinsicMatrix(2,2); % Pixel size along y-axis (meters)
  %% image center
  principalPointX = IntrinsicMatrix(1,3) * pixelSizeX; % (along x-axis)
  principalPointY = IntrinsicMatrix(2,3) * pixelSizeY; % (along y-axis)
  %% Image size - meters
  ImageWidthMeters = principalPointX * 2; % meters (along x-axis)
  ImageHeightMeters = principalPointY * 2; % meters (along y-axis)
  %% Image size - pixels
  ImageWidthPixels = ImageWidthMeters / pixelSizeX; % pixels (along x-axis)
  ImageHeightPixels = ImageHeightMeters / pixelSizeY; % pixels (along y-axis)
  %% Visualize in the reference frame
  upperLeftCorner = [ -ImageWidthMeters/2; -ImageHeightMeters/2; FocalLength ];
  upperRightCorner = [ ImageWidthMeters/2; -ImageHeightMeters/2; FocalLength ];
  lowerRightCorner = [ ImageWidthMeters/2; ImageHeightMeters/2; FocalLength ];
  lowerLeftCorner = [ -ImageWidthMeters/2; ImageHeightMeters/2; FocalLength ];
  imageBordersCamera = [ upperLeftCorner, upperRightCorner, lowerRightCorner, lowerLeftCorner, upperLeftCorner ];
  imageBordersMetricSpace = imageBordersCamera(1:2,:) + [principalPointX; principalPointY]; % Metric Space
  imageBordersSensorSpace = [imageBordersMetricSpace(1,:)/pixelSizeX; imageBordersMetricSpace(2,:)/pixelSizeY]; % Sensor Space
  %% Ploting
  plot( ImagePointsSensorSpace(1,:),  ImagePointsSensorSpace(2,:), 'go', 'markersize', 10, 'markerfacecolor', 'g' ); hold on;
  plot( ImagePointsSensorSpace(1,:),  ImagePointsSensorSpace(2,:), 'c', 'linewidth', 4 );
  xlabel(num2str(ImageWidthPixels),'fontsize',20);
  ylabel(num2str(ImageHeightPixels),'fontsize',20);
  axis ij;
  plot( imageBordersSensorSpace(1,:),  imageBordersSensorSpace(2,:), 'k', 'linewidth', 6 );
  xlim([0 ImageWidthPixels]); ylim([0 ImageHeightPixels]);
  xlabel(num2str(ImageWidthPixels),'fontsize',20);
  ylabel(num2str(ImageHeightPixels),'fontsize',20);
  grid on; title('Camera Image - 2D Sensor Space','fontsize',20);

end

