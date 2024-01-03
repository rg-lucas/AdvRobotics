%%%%Lucas Gomes - Homework 3 - RAFA_X1 Robot  - FVK
close all; clear; clc;
% end-effector pose
He0= eye(4);

qe=[0; 0; 0];
q7 = [-0.07725; 0; 0];
q6=[-0.07725-0.0415; 0; 0];
q5=[-0.07725-0.0415-0.079; 0; 0];
q4=[-0.07725-0.0415-0.079-0.0415; 0; 0];
q3=[-0.07725-0.0415-0.079-0.0415-0.0796; 0; 0];
q2=[-0.07725-0.0415-0.079-0.0415-0.0796; 0; -0.1175];
q1=[-0.07725-0.0415-0.079-0.0415-0.0796; 0; -0.1175-0.1445];
q0=q1;
q=[q0,q1,q2,q3,q4,q5,q6,q7,qe] ;

% joints screw axes
x=[1; 0; 0];
y=[0; 1; 0];
z=[0; 0; 1];


xi1 = [ z; cross(q1,(z))];
xi2 = [ -y; cross(q2,(-y))];
xi3 = [ -y; cross(q3,(-y))];
xi4 = [ x; cross(q4,(x))];
xi5 = [ -y; cross(q5,(-y))];
xi6 = [ x; cross(q6,(x))];
xi7 = [ z; cross(q7,(z))];

xi = [xi1, xi2, xi3, xi4, xi5, xi6, xi7];

%Jacobian Home configuration
J0=xi

disp(J0);

% joints values
theta0 = zeros (7,1); % Initial values are all 0
theta = [deg2rad(0); deg2rad(0); deg2rad(0); deg2rad(-90); deg2rad(0); deg2rad(90); deg2rad(90); ];
% Values change in axis 4 6 and 7, in -90, 90 and 90 degress

%The final values after the change will be the changing - the initial
delta_theta = theta - theta0;

n = 7;
He = He0;

% Making the changes
for i= n:-1:1
  xi_hat = twist2homogenousform(xi(:,i));
  He = expm(delta_theta(i)*xi_hat)*He;
endfor

He;


qe_prime = He*[qe;1];
qe_prime = qe_prime(1:3);

%joint 7
i=7;
Hi=eye(4);
for k= i-1:-1:1
  xi_hat = twist2homogenousform(xi(:,k));
  Hi = expm(delta_theta(k)*xi_hat)*Hi;
endfor


q7_prime = Hi*[q7;1];
q7_prime = q7_prime(1:3);
H_bar = Hpoint2Hline(Hi);
xi7_prime = H_bar*xi(:,7);

xi7_prime;

%joint 6
i = 6;
Hi = eye(4);

for k=i-1:-1:1
  xi_hat = twist2homogenousform(xi(:,k));
  Hi = expm(delta_theta(k)*xi_hat)*Hi;
endfor

q6_prime = Hi*[q6;1];
q6_prime = q6_prime(1:3);
H_bar = Hpoint2Hline(Hi);
xi6_prime = H_bar*xi(:,6);

%joint 5
i = 5;
Hi = eye(4);

for k=i-1:-1:1
  xi_hat = twist2homogenousform(xi(:,k));
  Hi = expm(delta_theta(k)*xi_hat)*Hi;
endfor

q5_prime = Hi*[q5;1];
q5_prime = q5_prime(1:3);
H_bar = Hpoint2Hline(Hi);
xi5_prime = H_bar*xi(:,5);

%joint 4
i = 4;
Hi = eye(4);

for k=i-1:-1:1
  xi_hat = twist2homogenousform(xi(:,k));
  Hi = expm(delta_theta(k)*xi_hat)*Hi;
endfor

q4_prime = Hi*[q4;1];
q4_prime = q4_prime(1:3);
H_bar = Hpoint2Hline(Hi);
xi4_prime = H_bar*xi(:,4);

%joint 3
i = 3;
Hi = eye(4);

for k=i-1:-1:1
  xi_hat = twist2homogenousform(xi(:,k));
  Hi = expm(delta_theta(k)*xi_hat)*Hi;
endfor

q3_prime = Hi*[q3;1];
q3_prime = q3_prime(1:3);
H_bar = Hpoint2Hline(Hi);
xi3_prime = H_bar*xi(:,3);

%joint 2
i = 2;
Hi = eye(4);

for k=i-1:-1:1
  xi_hat = twist2homogenousform(xi(:,k));
  Hi = expm(delta_theta(k)*xi_hat)*Hi;
endfor

q2_prime = Hi*[q2;1];
q2_prime = q2_prime(1:3);
H_bar = Hpoint2Hline(Hi);
xi2_prime = H_bar*xi(:,2);

%joint 1
i = 1;
Hi = eye(4);
q1_prime = q1;
xi1_prime = xi(:,1);

%base
q0_prime = q0;

%The new position vector
xi_prime = [xi1_prime,xi2_prime,xi3_prime,xi4_prime,xi5_prime,xi6_prime,xi7_prime];

%The new position Jacobian
J=xi_prime

%disp(J);


