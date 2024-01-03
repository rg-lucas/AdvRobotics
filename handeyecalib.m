%% Hand-Eye Calibration AX = XB.
%% "Hand-eye calibration using dual quaternions"
%% by Konstantinos Daniildis
function X = handeyecalib(A, B)
%% A and B contain the sequences of known n-transformations [4x4n]
%% X is the transformation between the sensor and a relatively fixed coordinate frame

% compute the number of known transformations
[m, n] = size(A); n = n/4;

% prepare the memory in advance for the T matrix in equation (33)
T= zeros(6*n, 8);

%% STEP-1 %%
for i = 1:n

  % extract the known transformations one by one
  A1 = A(:,4*i-3:4*i);
  B1 = B(:,4*i-3:4*i);

  % transform them into unit dual quaternion representation
  a_hat =  H442UDQ(A1);
  b_hat =  H442UDQ(B1);

  % extract vector parts from real and dual components of UDQS
  arv = a_hat(2:4);  adv = a_hat(6:8);
  brv = b_hat(2:4);  bdv = b_hat(6:8);

  % compose T matrix of the linear system from equations (31) and (33)
  T(6*i-5:6*i,:)= [ arv-brv, skew(arv+brv), zeros(3,4); ...
                    adv-bdv, skew(adv+bdv), arv-brv, skew(arv+brv) ];
end

%% STEP-2 %%
% solve using Singular Value Decomposition
[u,s,v]=svd(T);
u1 = v(1:4,7); v1 = v(5:8,7); % right-singular vector 7
u2 = v(1:4,8); v2 = v(5:8,8); % right-singular vector 8

%% STEP-3 %%

% coefficients of the 2nd order equation (35)
a = u1'*v1;
b = u1'*v2+u2'*v1;
c = u2'*v2;

% solutions of equation (35) for s when lambdal is replaced by s*lambda2
s1=(-b+sqrt(b^2-4*a*c))/(2*a);
s2=(-b-sqrt(b^2-4*a*c))/(2*a);
s=[s1; s2];

%% STEP-4 %%
% choose the solution which gives the largest value for the expression below
[val,in]= max(s.^2*(u1'*u1) + 2*s*(u1'*u2) + u2'*u2);
s = s(in); % the solution of the 2nd order equation

lambda2 = sqrt(1/val); % solve for lambda2 using equation (36)
lambda1 = s*lambda2; % compute lambda1 using s and lambda2

%% STEP-5 %%
% linear combination of right-singular vectors 7 and 8
x_hat = lambda1*v(:,7) + lambda2*v(:,8);

%the hand-eye calibration matrix in homomegeous coordinates
X = UDQ2H44( x_hat );

end
