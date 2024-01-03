function H = UDQ2H44( x_hat )
%% x_hat : 8x1 unit dual quaternion
% H : 4x4 homogenous transformation

  XR_bar = x_hat(1:4);

  if(XR_bar(1) != 1) % theta is different than zero
    theta = 2*acos(XR_bar(1));  % angle of rotation
    u = XR_bar(2:4)/norm(XR_bar(2:4));  % axis of rotation
    R = rodrigues( u, theta );
  else
    R = eye(3)
  endif

  t = translationfromUDQ( x_hat );

  H = [R,t; zeros(1,3), 1];

endfunction
