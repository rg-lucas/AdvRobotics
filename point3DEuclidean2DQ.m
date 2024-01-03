function pointDQ = point3DEuclidean2DQ( x )
  
  barX_R = [ 1; zeros(3,1) ];
  barX_D = [ 0; x ];
  pointDQ = [ barX_R; barX_D ];
  
endfunction