function met = pattern_pix2metric( pix, fx, fy, Ox, Oy )

for i = 1:4
   
    met(1,i) =  (pix(1,i) - Ox)/fx;
    met(2,i) =  (pix(2,i) - Oy)/fy;
    
end