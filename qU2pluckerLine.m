function L = qU2pluckerLine (q, u)
  % q = a point on the line
  % u = unit direction vector of the line
  % L = Euclidean normalized plucker line

  m = cross(q, u);
  L = [u;m];

end
