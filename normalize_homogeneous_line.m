% Euclidian normalization of a homogeneous line
function nL = normalize_homogeneous_line(homogeneous_line)
  a=homogeneous_line(1);
  b=homogeneous_line(2);
  c=homogeneous_line(3);

  cos_phi = a / sqrt(a^2+b^2);
  sin_phi = b / sqrt(a^2+b^2);
  d = c/sqrt(a^2+b^2);

  nL=[-sign(d)*cos_phi; -sign(d)*sin_phi; -sign(d)*d];

end
