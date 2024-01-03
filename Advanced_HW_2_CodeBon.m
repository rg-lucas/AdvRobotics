%%%% Adv Robotics - HW2 - LUCAS GOMES
%RAFA-X1 7 dof robot arm

clear; close all; clc;

% Home configuration

% joint locations in 3D Euclidean Coordinates
qe = [0; -79.6-41.5-79-41.5-77.25; 144.5+117.5 ]; %Sixth point
q6 = [0; -79.6-41.5-79-41.5; 144.5+117.5]; %Fifth point
q5 = [0; -79.6-41.5-79; 144.5+117.5]; %Fourth point
q4 = [0; -79.6-41.5; 144.5+117.5]; %Third point
q3 = [0; -79.6; 144.5+117.5];
q2 = [0; 0; 144.5+117.5]; %Second point
q1 = [0; 0; 144.5]; %first point
q0 = [0; 0; 0]; %Base
q = [q0, q1, q2, q3, q4, q5, q6, qe];
%Transforming all points to Dual Quaternials
qe_hat = point3DEuclidean2DQ( qe );
q6_hat = point3DEuclidean2DQ( q6 );
q5_hat = point3DEuclidean2DQ( q5 );
q4_hat = point3DEuclidean2DQ( q4 );
q3_hat = point3DEuclidean2DQ( q3 );
q2_hat = point3DEuclidean2DQ( q2 );
q1_hat = point3DEuclidean2DQ( q1 );
q0_hat = point3DEuclidean2DQ( q0 );


%%%%%%%%%%%Checking the points%%%%%%%%%%%%%%
%plotPointDQ (qe_hat);%plotPointDQ (q6_hat);
%plotPointDQ (q5_hat);%plotPointDQ (q4_hat);
%plotPointDQ (q3_hat);%plotPointDQ (q2_hat);
%plotPointDQ (q1_hat);%plotPointDQ (q0_hat);

%%%%% Creating the the axis in Plucker coordinates %%%%%%%%
X_axis = [1; 0; 0];
Y_axis = [0; 1; 0];
Z_axis = [0; 0; 1];
%%%%% Transforming them into a direction %%%%%%%%%%%%%%%%%%%
X_axis = X_axis/norm(X_axis);
Y_axis = Y_axis/norm(Y_axis);
Z_axis = Z_axis/norm(Z_axis);
%%%%% Transforming them with the robot points into a DQ %%%%
L1 = qU2pluckerLine( q1, Z_axis);
L2 = qU2pluckerLine( q2, Y_axis);
L3 = qU2pluckerLine( q3, Y_axis);
L4 = qU2pluckerLine( q4, Y_axis);
L5 = qU2pluckerLine( q5, Y_axis);
L6 = qU2pluckerLine( q6, Y_axis);
Le = qU2pluckerLine( qe, Y_axis);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ell_hat1 = linePlucker2DQ(L1);
ell_hat2 = linePlucker2DQ(L2);
ell_hat3 = linePlucker2DQ(L3);
ell_hat4 = linePlucker2DQ(L4);
ell_hat5 = linePlucker2DQ(L5);
ell_hat6 = linePlucker2DQ(L6);
ell_hate = linePlucker2DQ(Le);
%%%% Creating a Vector that joints all of the points and lines %%%%%
q_hat = [qe_hat, q6_hat, q5_hat, q4_hat, q3_hat, q2_hat, q1_hat, q0_hat];
L = [ell_hat1, ell_hat2, ell_hat3, ell_hat4, ell_hat5, ell_hat6, ell_hate];
%%%% Ploting the robot in DQ %%%%%%%%%
plot3( q_hat(6,:), q_hat(7,:), q_hat(8,:), 'LineWidth', 4, 'Color', 'k');
title('Robot in DQ', 'fontsize', 20 );
hold on;

for i=1:size(q_hat,2)
  plot3( q_hat(6,i), q_hat(7,i), q_hat(8,i), 'ro', 'markersize', 8, 'markerfacecolor', 'r');
end

plot3( q_hat(6,1), q_hat(7,1), q_hat(8,1), 'bo', 'markersize', 8, 'markerfacecolor', 'b');
plot3( q_hat(6,end), q_hat(7,end), q_hat(8,end), 'go', 'markersize', 8, 'markerfacecolor', 'g');

for k=1:size(L,2)
  plotLineDQatApoint( L(:,k), q_hat(:,k+1), 0.12, 'r' );
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Moving %%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% FPK (forward position kinematics)
% given joint configuration
He0 = eye(4);
theta0 = zeros(7,1);

theta = [ deg2rad(0); deg2rad(80); deg2rad(0); deg2rad(-70); deg2rad(0); deg2rad(-35); deg2rad(90) ];

n = 6;
He = He0;
delta_theta = theta - theta0;

% we multiply by starting from the last joint towards the first foint

for i = n:-1:1

  xi_hat = twist2homogenousform( q(:,i) );
  He = expm( delta_theta(i)*xi_hat )*He;

endfor

% End-effector pose
He0 = eye(4);
He,

% robot's geometry at its new configuration
% end-effector position

qe_prime = He*[qe;1]; qe_prime = qe_prime(1:3);

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


