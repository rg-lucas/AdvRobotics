%%%% Adv Robotics - HW2 - LUCAS GOMES
%RAFA-X1 7 dof robot arm

clear; close all; clc;

% Home configuration
% end-effector pose
He0 = eye(4);

% joint locations


qe = -[0;79.6+41.5+79+41.5+77.25; 144.5+117.5 ]; %Sixth point
q6 = -[0; 79.6+41.5+79+41.5; 144.5+117.5]; %Fifth point
q5 = -[0; 79.6+41.5+79; 144.5+117.5]; %Fourth point
q4 = -[0; 79.6+41.5; 144.5+117.5]; %Third point
q3 = -[0; 79.6; 144.5+117.5];
q2 = -[0; 0; 144.5+117.5]; %Second point
q1 = -[0; 0; 144.5]; %first point
q0 = -[0; 0; 0]; %Base

q = [q0,q1,q2,q3,q4,q5,q6,qe]

% joints screw axes (lines of rotation)
x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];

xi1 = [-z; cross(q1,(-z)) ];
xi2 = [-x; cross(q2,(-x)) ];
xi3 = [-x; cross(q3,(-x)) ];
xi4 = [ y; cross(q4,y) ];
xi5 = [-x; cross(q5,(-x)) ];
xi6 = [ y; cross(q6,y) ];

xi = [xi1,xi2,xi3,xi4,xi5,xi6];
% joint values
theta0 = zeros(6,1);

% plotting robot at home/start configuration

figure;
plotRobot(q, xi);
plotCoordinateFramefromPose( He0, 0.1 );
set (gca (), "zdir", "reverse"); axis equal;
grid on;
title('Home configuration','fontsize',20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Fazendo as transformadas ponto a ponto %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initial point coordinates
pointDQ0 = point3DEuclidean2DQ(q0)
pointDQ1 = point3DEuclidean2DQ(q1)
pointDQ2 = point3DEuclidean2DQ(q2)
pointDQ3 = point3DEuclidean2DQ(q3)
pointDQ4 = point3DEuclidean2DQ(q4)
pointDQ5 = point3DEuclidean2DQ(q5)
pointDQ6 = point3DEuclidean2DQ(q6)
pointDQe = point3DEuclidean2DQ(qe)

%final point coordinates
x0 = [0; 2; 3]
x1 = [0; 2; 3]
x2 = [0; 2; 3]
x3 = [0; 2; 3]
x4 = [0; 2; 3]
x5 = [0; 2; 3]
x6 = [0; 2; 3]
xe = [0; 2; 3]

x_hat = dq_from_tutheta(x, x, 3)

x_hat_2stars = [  quartenionConjugateforQ(x_hat(1:4));
                  -quartenionConjugateforQ(x_hat(5:8))        ]

pointDQlineB = dqprod(x_hat, pointDQ)
pointDQline = dqprod(pointDQlineB, x_hat_2stars)



% plotting robot at Prime configuration
figure;
plotRobot(q_prime, xi_prime );
plotCoordinateFramefromPose( He, 0.1 );
set (gca (), "zdir", "reverse"); axis equal;
axis([-70 70 -70 70 -70 70]); grid on;
title('Touching the ground configuration','fontsize',20);

