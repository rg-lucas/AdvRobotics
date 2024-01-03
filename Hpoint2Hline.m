function H6x6 = Hpoint2Hline( H4x4 )

  R = H4x4(1:3, 1:3); t = H4x4(1:3,4);

  H6x6 = [ R,    zeros(3,3);
          skew(t)*R,   R  ];

end


