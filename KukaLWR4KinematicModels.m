%%% Lucas Resende Gomes - Homework 4
%%% KukaLWR4KinematicModels

 function [He, xi_prime, q_prime] = KukaLWR4KinematicModels(theta)
%Home Configuration
% joint locations
n = 7; % number of active joints
qe = [ 0; 0; 0 ];
q7 = [ 0; 0; 0 ];
q6 = [ 0; 0; 0 ];
q5 = [ 0; 0; -0.185 ];
q4 = [ 0; 0; -0.185-0.185 ];
q3 = [ 0; 0; -0.185-0.185-0.2 ];
q2 = [ 0; 0; -0.185-0.185-0.2-0.2 ];
q1 = [ 0; 0; -0.185-0.185-0.2-0.2-0.155 ];
q0 = [ 0; 0; -0.185-0.185-0.2-0.2-0.155-0.155 ];

q = [q0,q1,q2,q3,q4,q5,q6,q7,qe];

%Pose (home configuration)
He = eye(4);

%Unit vectors
x = [ 1; 0; 0 ];
y = [ 0; 1; 0 ];
z = [ 0; 0; 1 ];

%Axis of rotation of the joints
xi1 = [z;   cross(q1, z  )  ];
xi2 = [-x;  cross(q2,(-x))  ];
xi3 = [z;   cross(q3, z  )  ];
xi4 = [x;   cross(q4, x  )  ];
xi5 = [z;   cross(q5, z  )  ];
xi6 = [-x;  cross(q6,(-x))  ];
xi7 = [ z;  cross(q7, z  )  ];

xi = [xi1,xi2,xi3,xi4,xi5,xi6,xi7]; % Jacobian (home configuration)

for i = n:-1:1

  xi_hat = twist2homogenousform( xi(:,i) );
  He = expm( theta(i)*xi_hat )*He;

end

% robot's geometry at its new configuration
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
