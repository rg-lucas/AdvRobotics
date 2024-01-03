function H = randomH44()

  theta = rand;
  u = rand(3,1);
  u_unit = u/norm(u);
  R = rodrigues( u_unit, theta );

  t = rand(3,1);

  H = [   R,    t;
       0, 0, 0, 1];

endfunction
