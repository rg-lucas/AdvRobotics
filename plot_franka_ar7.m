function plot_franka_ar7( theta )

  % theta: joint angles

  % Joint positions
  qe = [ 0; 0; 0 ];
  q7 = [ 0; 0; -0.107 ];
  q6 = [ -0.088; 0; -0.107 ];
  q5 = q6;
  q4 = [ 0; 0; -0.107 + 0.384 ];
  q3 = [ -0.088; 0; -0.107 + 0.384 ];
  q2 = [ -0.088; 0; -0.107 + 0.384 + 0.316 ];
  q1 = q2;
  q0 = [ -0.088; 0; -0.107 + 0.384 + 0.316 + 0.333 ];

  q = [q0,q1,q2,q3,q4,q5,q6,q7,qe];

  % Unit vectors
  x = [ 1; 0; 0 ];
  y = [ 0; 1; 0 ];
  z = [ 0; 0; 1 ];

  % Axes twist
  xi1 = [ -z; cross(q1,(-z)) ];
  xi2 = [ -y; cross(q2,(-y)) ];
  xi3 = [ -z; cross(q3,(-z)) ];
  xi4 = [ y; cross(q4,y) ];
  xi5 = [ -z; cross(q5,(-z)) ];
  xi6 = [ y; cross(q6,(y)) ];
  xi7 = [ z; cross(q7,z) ];

  xi = [xi1,xi2,xi3,xi4,xi5,xi6,xi7]; % Jacobian for home configuration

  % Plotting

  % end-effector
  He = eye(4);

  for i = 7:-1:1

  xi_hat = twist2homogenousform( xi(:,i) );
  He = expm( theta(i)*xi_hat )*He;

  end

  qe_prime = He*[qe;1];
  qe_prime = qe_prime(1:3);

  % joint 7
  Hi = eye(4);
  for k = 6:-1:1
    xi_hat = twist2homogenousform( xi(:,k) );
    Hi = expm( theta(k)*xi_hat )*Hi;
  end

  q7_prime = Hi*[q7; 1];
  q7_prime = q7_prime(1:3);
  H_bar = Hpoint2Hline( Hi );
  xi7_prime = H_bar*xi(:,7);

  % joint 6
  Hi = eye(4);
  for k = 5:-1:1
    xi_hat = twist2homogenousform( xi(:,k) );
    Hi = expm( theta(k)*xi_hat )*Hi;
  end

  q6_prime = Hi*[q6; 1];
  q6_prime = q6_prime(1:3);
  H_bar = Hpoint2Hline( Hi );
  xi6_prime = H_bar*xi(:,6);

  % joint 5
  Hi = eye(4);
  for k = 4:-1:1
    xi_hat = twist2homogenousform( xi(:,k) );
    Hi = expm( theta(k)*xi_hat )*Hi;
  end

  q5_prime = Hi*[q5; 1];
  q5_prime = q5_prime(1:3);
  H_bar = Hpoint2Hline( Hi );
  xi5_prime = H_bar*xi(:,5);

  % joint 4
  Hi = eye(4);
  for k = 3:-1:1
    xi_hat = twist2homogenousform( xi(:,k) );
    Hi = expm( theta(k)*xi_hat )*Hi;
  end

  q4_prime = Hi*[q4; 1];
  q4_prime = q4_prime(1:3);
  H_bar = Hpoint2Hline( Hi );
  xi4_prime = H_bar*xi(:,4);

  % joint 3
  Hi = eye(4);
  for k = 2:-1:1
    xi_hat = twist2homogenousform( xi(:,k) );
    Hi = expm( theta(k)*xi_hat )*Hi;
  end

  q3_prime = Hi*[q3; 1];
  q3_prime = q3_prime(1:3);
  H_bar = Hpoint2Hline( Hi );
  xi3_prime = H_bar*xi(:,3);

  % joint 2
  Hi = eye(4);
  for k = 1:-1:1
    xi_hat = twist2homogenousform( xi(:,k) );
    Hi = expm( theta(k)*xi_hat )*Hi;
  end

  q2_prime = Hi*[q2; 1];
  q2_prime = q2_prime(1:3);
  H_bar = Hpoint2Hline( Hi );
  xi2_prime = H_bar*xi(:,3);

  % joint 1
  i = 1;
  Hi = eye(4);
  q1_prime = q1;
  xi1_prime = xi(:,1);

  % joint 0
  q0_prime = q0;

  q_prime = [q0_prime, q1_prime, q2_prime, q3_prime, q4_prime, q5_prime, q6_prime, q7_prime, qe_prime];
  xi_prime = [xi1_prime, xi2_prime, xi3_prime, xi4_prime, xi5_prime, xi6_prime, xi7_prime];

  hold on;
  plotRobot( q_prime, xi_prime );
  plotCoordinateFramefromPose( He, 0.1 );
  set (gca (), 'zdir', 'reverse');
  grid on;
  axis equal;
  view( -45, 30 );
  axis( [ -1 1 -1 1 -0.5 1 ] );

endfunction
