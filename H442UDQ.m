function x_hat = H442UDQ( H )

  R = H(1:3,1:3);
  t = H(1:3,4);

  S = logm(R);
  w = skew2w( S );
  theta = norm(w);
  u = w/theta;

  x_hat = dq_from_tutheta( t, u, theta );

 endfunction
