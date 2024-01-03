function z_hat = dqprod( x_hat, y_hat)
  XR_bar = x_hat(1:4); XD_bar = x_hat(5:8);
  YR_bar = y_hat(1:4); YD_bar = y_hat(5:8);

  z_hat = [ qprod( XR_bar, YR_bar); % real part
            qprod( XR_bar, YD_bar) + qprod( XD_bar, YR_bar) ];  % dual part

end


