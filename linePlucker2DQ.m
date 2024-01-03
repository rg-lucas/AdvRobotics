function ell_hat = linePlucker2DQ( ell )
  
  u = ell(1:3);
  m = ell(4:6);
  
  barX_R = [ 0; u ];
  barX_D = [ 0; m ];
  
  ell_hat = [ barX_R; barX_D ];
  
endfunction