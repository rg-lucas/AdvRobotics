% quartenion product
function x_bar = qprod( x1_bar, x2_bar)
  s1 = x1_bar(1); v1 = x1_bar(2:4);
  s2 = x2_bar(1); v2 = x2_bar(2:4);

  x_bar = [     s1*s2 - v1'*v2;
            s1*v2 + s2*v1 + cross(v1,v2) ];
end


