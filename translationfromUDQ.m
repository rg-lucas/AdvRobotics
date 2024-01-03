function t = translationfromUDQ( X_hat )
  XR_bar = X_hat(1:4);
  XD_bar = X_hat(5:8);

  st = qprod(XD_bar, quartenionConjugateforQ(XR_bar) );
  t = 2*st(2:4);

end

