%%%% TP LUCAS GOMES and Antonio Marques
%Franka robot
clear; close all; clc;

% Home configuration
% end-effector pose
He0 = eye(4);

% joint locations


qe = [0; 0; -71-39]; %Seventh point
q7 = [0; 8;-71-39]; %Sixth point
q6 = [0; 4; -71-39/2]; %Fifth point
q5 = [0; 0; -71]; %Fourth point
q4 = [0; 10; -71]; %Third point
q3 = [0; 10; -51];
q2 = [0; 10; -31]; %Second point
q1 = [0; 0; -31]; %first point
q0 = [0; 0; 0]; %Base

q = [q0,q1,q2,q3,q4,q5,q6,q7,qe]

% joints screw axes (lines of rotation)
x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];

xi1 = [-z; cross(q1,(-z)) ];
xi2 = [-y; cross(q2,(-y)) ];
xi3 = [-z; cross(q3,(-z)) ];
xi4 = [ y; cross(q4,y) ];
xi5 = [-z; cross(q5,(-z)) ];
xi6 = [ y; cross(q6,y) ];
xi7 = [ y; cross(q7,y) ];

xi = [xi1,xi2,xi3,xi4,xi5,xi6,xi7];
% joint values
theta0 = zeros(7,1);

% FPK (forward position kinematics) of Franka robot
% given joint configuration

theta = [ deg2rad(0); deg2rad(80); deg2rad(0); deg2rad(-70); deg2rad(0); deg2rad(-35); deg2rad(90) ];

n = 7;
He = He0;
delta_theta = theta - theta0;

% we multiply by starting from the last joint towards the first foint

for i = n:-1:1

  xi_hat = twist2homogenousform( xi(:,i) );
  He = expm( delta_theta(i)*xi_hat )*He;

endfor

% End-effector pose

He,

% robot's geometry at its new configuration
% end-effector position

qe_prime = He*[qe;1]; qe_prime = qe_prime(1:3);

% position and screw axis of joint 7

i = 7;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
endfor

q7_prime = Hi*[q7; 1]; q7_prime = q7_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi7_prime = H_bar*xi(:,7);

% position and screw axis of joint 6
i = 6;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
endfor

q6_prime = Hi*[q6; 1]; q6_prime = q6_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi6_prime = H_bar*xi(:,6);

% position and screw axis of joint 5
i = 5;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
endfor

q5_prime = Hi*[q5; 1]; q5_prime = q5_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi5_prime = H_bar*xi(:,5);

% position and screw axis of joint 4
i = 4;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
endfor

q4_prime = Hi*[q4; 1]; q4_prime = q4_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi4_prime = H_bar*xi(:,4);

% position and screw axis of joint 3
i = 3;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
endfor

q3_prime = Hi*[q3; 1]; q3_prime = q3_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi3_prime = H_bar*xi(:,3);

% position and screw axis of joint 2
i = 2;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
endfor

q2_prime = Hi*[q2; 1]; q2_prime = q2_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi2_prime = H_bar*xi(:,2);

% position and screw axis of joint 1
i = 1;
Hi = eye(4);
q1_prime = q1;
xi1_prime = xi(:,1);

% position of robot's base
q0_prime = q0;

% position vector
q_prime = [q0_prime, q1_prime, q2_prime, q3_prime, q4_prime, q5_prime, q6_prime, q7_prime, qe_prime]

% screw axes vector
xi_prime = [xi1_prime, xi2_prime, xi3_prime, xi4_prime, xi5_prime, xi6_prime, xi7_prime]

% plotting robot at home/start configuration

figure;
plotRobot(q, xi);
plotCoordinateFramefromPose( He0, 0.1 );
set (gca (), "zdir", "reverse"); axis equal;
axis([-150 150 -150 150 -150 150]); grid on;
title('Home configuration','fontsize',20);

% plotting robot at Prime configuration
figure;
plotRobot(q_prime, xi_prime );
plotCoordinateFramefromPose( He, 0.1 );
set (gca (), "zdir", "reverse"); axis equal;
axis([-70 70 -70 70 -70 70]); grid on;
title('Touching the ground configuration','fontsize',20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5




figure;
%pause(3);
for i=0:200
  if i<101, t = 1;
% FPK (forward position kinematics) of Franka robot
% given joint configuration
clf;
theta = [ deg2rad(0); deg2rad(1.6*i/2); deg2rad(0); deg2rad(-1.4*i/2); deg2rad(0); deg2rad(-0.7*i/2); deg2rad(1.8*i/2) ];
n = 7;
He = He0;
delta_theta = theta - theta0;

% we multiply by starting from the last joint towards the first foint

for i = n:-1:1

  xi_hat = twist2homogenousform( xi(:,i) );
  He = expm( delta_theta(i)*xi_hat )*He;

end

% End-effector pose

He,

% robot's geometry at its new configuration
% end-effector position

qe_prime = He*[qe;1]; qe_prime = qe_prime(1:3);

% position and screw axis of joint 7

i = 7;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
end

q7_prime = Hi*[q7; 1]; q7_prime = q7_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi7_prime = H_bar*xi(:,7);

% position and screw axis of joint 6
i = 6;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
end

q6_prime = Hi*[q6; 1]; q6_prime = q6_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi6_prime = H_bar*xi(:,6);

% position and screw axis of joint 5
i = 5;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
end

q5_prime = Hi*[q5; 1]; q5_prime = q5_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi5_prime = H_bar*xi(:,5);

% position and screw axis of joint 4
i = 4;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
end

q4_prime = Hi*[q4; 1]; q4_prime = q4_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi4_prime = H_bar*xi(:,4);

% position and screw axis of joint 3
i = 3;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
end

q3_prime = Hi*[q3; 1]; q3_prime = q3_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi3_prime = H_bar*xi(:,3);

% position and screw axis of joint 2
i = 2;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
end

q2_prime = Hi*[q2; 1]; q2_prime = q2_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi2_prime = H_bar*xi(:,2);

% position and screw axis of joint 1
i = 1;
Hi = eye(4);
q1_prime = q1;
xi1_prime = xi(:,1);

% position of robot's base
q0_prime = q0;

% position vector
q_prime = [q0_prime, q1_prime, q2_prime, q3_prime, q4_prime, q5_prime, q6_prime, q7_prime, qe_prime]

% screw axes vector
xi_prime = [xi1_prime, xi2_prime, xi3_prime, xi4_prime, xi5_prime, xi6_prime, xi7_prime]

% plotting robot at home configuration
plotRobot(q_prime, xi_prime );
hold on;
plotCoordinateFramefromPose( He0, 0.1 );
hold on;
set (gca (), 'zdir', 'reverse'); axis equal;
axis([-150 150 -150 150 -150 150]); grid on;
title('Animation going to touch the ground','fontsize',20);
drawnow; hold on;

else
clf;
theta2 = [ deg2rad(0); deg2rad(-1.6*(i-100)/2); deg2rad(0); deg2rad(1.4*(i-100)/2); deg2rad(0); deg2rad(0.7*(i-100)/2); deg2rad(1.8*(i-100)/2) ];
n = 7;
He = He0;
delta_theta2 = theta + theta2;

% we multiply by starting from the last joint towards the first foint

for i = n:-1:1

  xi_hat = twist2homogenousform( xi(:,i) );
  He = expm( delta_theta2(i)*xi_hat )*He;

end

% End-effector pose

He,

% robot's geometry at its new configuration
% end-effector position

qe_prime = He*[qe;1]; qe_prime = qe_prime(1:3);

% position and screw axis of joint 7

i = 7;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta2(k)*xi_hat )*Hi;
end

q7_prime = Hi*[q7; 1]; q7_prime = q7_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi7_prime = H_bar*xi(:,7);

% position and screw axis of joint 6
i = 6;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta2(k)*xi_hat )*Hi;
end

q6_prime = Hi*[q6; 1]; q6_prime = q6_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi6_prime = H_bar*xi(:,6);

% position and screw axis of joint 5
i = 5;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta2(k)*xi_hat )*Hi;
end

q5_prime = Hi*[q5; 1]; q5_prime = q5_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi5_prime = H_bar*xi(:,5);

% position and screw axis of joint 4
i = 4;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta2(k)*xi_hat )*Hi;
end

q4_prime = Hi*[q4; 1]; q4_prime = q4_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi4_prime = H_bar*xi(:,4);

% position and screw axis of joint 3
i = 3;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta2(k)*xi_hat )*Hi;
end

q3_prime = Hi*[q3; 1]; q3_prime = q3_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi3_prime = H_bar*xi(:,3);

% position and screw axis of joint 2
i = 2;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta2(k)*xi_hat )*Hi;
end

q2_prime = Hi*[q2; 1]; q2_prime = q2_prime(1:3);
H_bar = Hpoint2Hline( Hi );
xi2_prime = H_bar*xi(:,2);

% position and screw axis of joint 1
i = 1;
Hi = eye(4);
q1_prime = q1;
xi1_prime = xi(:,1);

% position of robot's base
q0_prime = q0;

% position vector
q_prime = [q0_prime, q1_prime, q2_prime, q3_prime, q4_prime, q5_prime, q6_prime, q7_prime, qe_prime]

% screw axes vector
xi_prime = [xi1_prime, xi2_prime, xi3_prime, xi4_prime, xi5_prime, xi6_prime, xi7_prime]

% plotting robot at home configuration
plotRobot(q_prime, xi_prime );
hold on;
plotCoordinateFramefromPose( He0, 0.1 );
hold on;
set (gca (), 'zdir', 'reverse'); axis equal;
axis([-150 150 -150 150 -150 150]); grid on;
title('Animation comming back to original position','fontsize',20);
drawnow; hold on;

endif

end

%sdsdsd

