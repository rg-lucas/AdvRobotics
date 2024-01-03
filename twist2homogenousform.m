function xi_hat = twist2homogenousform(xi)
% xi: unit twist
% wi_hat: homogenous form of the unit twist

w = xi(1:3);
v = xi(4:6);
xi_hat = [ skew(w), v; zeros(1,4)];

end