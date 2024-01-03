% unit dual quartenion pose from position and axis-angle (orientation)
function x_hat = dq_from_tutheta(t, u, theta)
  %t : the position vectorize
  %u : unit axis vector of rotation
  %theta : angle of orientation in radians

  q_bar_orientation = [ cos(theta/2); sin(theta/2)*u ];

  XR_bar = [ 1; zeros(3,1) ];
  XD_bar = 0.5*[0;t];

  x_hat = [ qprod( XR_bar, q_bar_orientation);
            qprod( XD_bar, q_bar_orientation) ];
  end
