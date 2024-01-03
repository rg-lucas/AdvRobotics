function [He, xi_prime, q_prime] = RAFAX1KinematicModels(theta)

% Home configuration
% end-effector pose
He0 = eye(4);

% joint locations
n = 7; % number of active joints
qe = [0;0;0];
q7 = [-.07725;0;0];
q6 = [-.07725-.0415;0;0];
q5 = [-.07725-.0415-.079;0;0];
q4 = [-.07725-.0415-.079-.0415;0;0];
q3 = [-.07725-.0415-.079-.0415-.0796;0;0];
q2 = [-.07725-.0415-.079-.0415-.0796;0;-.1175];
q1 = [-.07725-.0415-.079-.0415-.0796;0;-.1175-.1445];
q0 = q1;


q = [q0,q1,q2,q3,q4,q5,q6,q7,qe];

% Unit vectors
x = [1; 0; 0]; y = [0; 1; 0]; z = [0; 0; 1];

% Twists of the joints
xi1 = [ z; cross(q1,(z))  ];
xi2 = [-y; cross(q2,(-y)) ];
xi3 = [-y; cross(q3,(-y)) ];
xi4 = [ x; cross(q4,x)    ];
xi5 = [-y; cross(q5,(-y)) ];
xi6 = [ x; cross(q6,x)    ];
xi7 = [ z; cross(q7,z)    ];

xi = [xi1,xi2,xi3,xi4,xi5,xi6,xi7];


% FPK (forward position kinematics) of Franka robot

He = He0;

% we multiply by starting from the last joint towards the first foint

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
