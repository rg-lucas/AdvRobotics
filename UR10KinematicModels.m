%%% Lucas Resende Gomes - Homework 7
%%% UR10KinematicModels

function [He, xi_prime, q_prime] = UR10KinematicModels(theta)
%Home Configuration
%Joint positions
n = 7;
qe = [ 0; 0; 0 ];
q7 = [ -0.0825; 0; 0 ];
q6 = [ -0.0825; 0; -0.09475 ];
q5 = [ -0.0825-0.1093; 0; -0.09475 ];
q4 = [ -0.0825-0.1093; 0; -0.09475-0.392 ];
q3 = [ -0.0825; 0; -0.09475-0.392 ];
q2 = [ -0.0825; 0; -0.09475-0.392-0.425 ];
q1 = [ -0.0825; 0; -0.09475-0.392-0.425-0.0892 ];
q0 = [ -0.0825-0.1093; 0; -0.09475-0.392-0.425-0.0892 ];

q = [q0,q1,q2,q3,q4,q5,q6,q7,qe];

%Pose (home configuration)
He0 = eye(4);

%Unit vectors
x = [ 1; 0; 0 ];
y = [ 0; 1; 0 ];
z = [ 0; 0; 1 ];

%Axis of rotation of the joints
xi1 = [ z; cross(q1,(z)) ];
xi2 = [ -x; cross(q2,(-x)) ];
xi3 = [ z; cross(q3,(z)) ];
xi4 = [ x; cross(q4,x) ];
xi5 = [ z; cross(q5,(z)) ];
xi6 = [ -x; cross(q6,(-x)) ];
xi7 = [ z; cross(q7,z) ];

xi = [xi1,xi2,xi3,xi4,xi5,xi6,xi7]; % Jacobian (home configuration)

% FPK (forward position kinematics) of UR10 robot
He = He0;
% we multiply by starting from the last joint towards the first joint
for i = n:-1:1

  xi_hat = twist2homogenousform( xi(:,i) );
  He = expm( theta(i)*xi_hat )*He;

end
% end-effector position
qe_prime = He*[qe;1]; q_prime(:,n+2) = qe_prime(1:3);

% position and screw axis of joints
for i = n:-1:1;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( theta(k)*xi_hat )*Hi;
end

q_prime_temp = Hi*[q(:,i+1); 1]; q_prime(:,i+1) = q_prime_temp(1:3);
H_bar = Hpoint2Hline( Hi );
xi_prime(:,i) = H_bar*xi(:,i);
end

% position of robot's base
q_prime(:,1) = q0;

%'End effector pose at Final configuration:'; He;
%'Jacobian Matrix at Final configuration:'; xi_prime;
end
