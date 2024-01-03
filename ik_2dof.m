%ik for robot
function theta = ik_2dof(x,y, lenL1, lenL2)

epslon = atan2(y,x);
beta = acos(((lenL1^2)+(lenL2^2)-(x^2)-(y^2))/(2*lenL1*lenL2));
alpha = acos(((x^2)+(y^2)+(lenL1^2)-(lenL2^2))/(2*lenL1*hypot(x,y)));

theta1 = rad2deg(epslon - alpha);
theta2 = rad2deg(pi - beta);

theta = [theta1;theta2];

end

