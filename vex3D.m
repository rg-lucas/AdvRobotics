function xi = vex3D( xi_hat )

  v = xi_hat(1:3,4);
  w = skew2w(xi_hat(1:3,1:3));

  xi = [w;v];

end
