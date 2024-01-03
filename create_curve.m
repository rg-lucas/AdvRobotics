function [curve, R] = create_curve(A, B, O, n)
% A and B are the endpoints of the curve
% O is the center of the circle
% n is the number of points to interpolate

% Calculate the distance between A and B
AB = norm(B - A);

% Calculate the height of the triangle formed by O, A, and B
H = norm(cross(A - O, B - O)) / norm(B - A);

% Calculate the midpoint of A and B
M = (A + B) / 2;

% Calculate the radius R as the hypotenuse of the right triangle formed by M and H
R = norm([norm(M - O), H]);

% First, compute the unit vector in the direction of AB
u = (B - A) / AB;

% Next, compute the vector from O to A
v = A - O;

% Compute the projection of v onto the plane perpendicular to u
w = v - dot(v, u) * u;

% Compute the length of w
lw = norm(w);

% Compute the center of the circle C
C = O + w - sqrt(R^2 - lw^2) * u;

% Compute the angle between OA and OC
theta = atan2(norm(cross(O-A,C-O)), dot(O-A,C-O));

% Compute the angle between the vectors OA and OB
phi = atan2(norm(cross(O-A,B-A)), dot(O-A,B-A));

% Create an array of angles for the interpolated points
angles = linspace(theta, phi, n);

% Compute the interpolated points
curve = zeros(length(O), n);
for i = 1:n
    % Compute the direction of the current point
    dir = (C - O) / norm(C - O);

    % Compute the position of the current point
    curve(:, i) = O' + R * cos(angles(i)) * dir' + R * sin(angles(i)) * cross(u, dir)';
end

end
