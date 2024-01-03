% Kuka 7dof robot by Danyela Macedo & Kaue Cavalcante
clear; close all; clc;

% Home configuration
% end-effector pose
He0 = eye(4);

% joint locations
qe = [0; 0; .71+.49];
q7 = [0; 0; .71+.45];
q6 = [0; 0; .71+.39];
q5 = [0; 0; .71+.39/2];
q4 = [0; 0; .71];
q3 = [0; 0.1; .71];
q2 = [0; 0.1; .31];
q1 = [0; 0; .31];
q0 = [0; 0; 0];


q = [q0,q1,q2,q3,q4,q5,q6,q7,qe];

% joints screw axes (lines of rotation)
x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];

xi1 = [-z; cross(q1,(-z)) ];
xi2 = [-y; cross(q2,(-y)) ];
xi3 = [-z; cross(q3,(-z)) ];
xi4 = [ y; cross(q4,y) ];
xi5 = [-z; cross(q5,(-z)) ];
xi6 = [-y; cross(q6,(-y)) ];
xi7 = [ z; cross(q7,z) ];

xi = [xi1,xi2,xi3,xi4,xi5,xi6,xi7];

% joint values
theta0 = zeros(7,1);

% FPK (forward position kinematics) of Franka robot
% given joint configuration
theta = [ deg2rad(0); deg2rad(-80); deg2rad(0); deg2rad(65); deg2rad(-0); deg2rad(35); deg2rad(0) ];

% Setting up the animation

qe_primes = [];
figure;
pause(3);
theta1=theta;
s =25; %animation step

for i=0:s*2
clf;
% if statement to make reverse trajectory
if (i >= s)
  i = s-(i-s);
endif
theta = i*(theta1/s);

n = 7;
He = He0;
delta_theta = theta - theta0;

% we multiply by starting from the last joint towards the first foint

for i = n:-1:1

  xi_hat = twist2homogenousform( xi(:,i) );
  He = expm( delta_theta(i)*xi_hat )*He;

end

He
% robot's geometry at its new configuration
% end-effector position

qe_prime = He*[qe;1]; qe_prime = qe_prime(1:3);
qe_primes = [qe_primes, qe_prime];

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

q0_prime = q0;
q_prime = [q0_prime, q1_prime, q2_prime, q3_prime, q4_prime, q5_prime, q6_prime, q7_prime, qe_prime];
xi_prime = [xi1_prime, xi2_prime, xi3_prime, xi4_prime, xi5_prime, xi6_prime, xi7_prime];

plotRobot(q_prime, xi_prime );
plot3(qe_primes(1,:), qe_primes(2,:), qe_primes(3,:),'g','linewidth',4);
plotCoordinateFramefromPose( He0, 0.1 );
set (gca (), 'zdir', 'normal'); axis equal;
axis([-1 1.2 -1 1.2 0 1.2]); grid on;
title('Robot trajectory animation','fontsize',20);
drawnow; pause(0.1);

end
