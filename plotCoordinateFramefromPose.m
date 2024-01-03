function plotCoordinateFramefromPose( H, s )
% H: 4x4 pose matrix
% s: scale for plotting

q = H(1:3,4);
x = H(1:3,1);
y = H(1:3,2);
z = H(1:3,3);

qx = q + s*x;
qy = q + s*y;
qz = q + s*z;

hold on;
plot3( [q(1) qx(1)], [q(2) qx(2)], [q(3) qx(3)], 'r', 'linewidth', 4 ); % x
plot3( [q(1) qy(1)], [q(2) qy(2)], [q(3) qy(3)], 'g', 'linewidth', 4 ); % y
plot3( [q(1) qz(1)], [q(2) qz(2)], [q(3) qz(3)], 'b', 'linewidth', 4 ); % z  

end

