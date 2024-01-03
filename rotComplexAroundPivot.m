function z = rotComplexAroundPivot(z0, p, phi)
  % Complex plane analysis
  % Rotating a line about a pivot point

  % Translate the line
  % such that pivot point is at the origin
  z = z0 + (z0 / (norm(z0)^2)) * real(conj(z0)*(-p));

  % Rotate the line about the origin by phi
  z = exp(i*phi)*z;

  % Tranlate the line such that pivot point
  % returns to its initial position
  z = z + (z / (norm(z)^2)) * real(conj(z)*p);
end
