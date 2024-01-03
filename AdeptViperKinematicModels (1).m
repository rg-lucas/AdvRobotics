% Adept Viper 6 dof robot by Gabriel Dias Flores & Kaue Santos Cavalcante
function [He, xi_prime, q_prime] = AdeptViperKinematicModels(theta)

% Home configuration
% end-effector pose
He0 = eye(4);

% joint locations
n = 6; % number of axis of rotations
qe = [0;            0;      0                     ];
q6 = qe;
q5 = [0;            0;      0.0645                ];
q4 = [-0.288;       0;      0.0645                ];
q3 = [-0.288;       0;      0.0645-.08            ];
q2 = [-0.288;       0;      0.0645-.08-.26        ];
q1 = [-0.288-0.08;  0;      0.0645-.08-.26        ];
q0 = [-0.288-0.08;  0;      0.0645-.08-.26-.203   ];

q = [q0,q1,q2,q3,q4,q5,q6,qe];

% Unit vectors
x = [1; 0; 0]; y = [0; 1; 0]; z = [0; 0; 1];

% Twists of the joints
xi1 = [z; cross(q1,(z)) ];
xi2 = [y; cross(q2,(y)) ];
xi3 = [y; cross(q3,(y)) ];
xi4 = [x; cross(q4,(x)) ];
xi5 = [y; cross(q5,(y)) ];
xi6 = [z; cross(q6,(z)) ];

xi = [xi1,xi2,xi3,xi4,xi5,xi6];


% FPK (forward position kinematics) of Viper robot

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
