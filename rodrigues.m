% axis-angle formula of rotation
function R = rodrigues( u, theta )
% u : unit rotation axis
% theta : rotation angle (radians) about the unit rotation axis

R = eye(3) + sin(theta)*skew(u) + (1-cos(theta))*skew(u)^2;

end