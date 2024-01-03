function [x0e, kuka_wire] = fkm_visual( q )
% q : is a vector of joint angles in radian
% x : is the end-effector pose

theta_offset = 0;
u_offset = [ 0; 0; 1];
d_offset = 1.215;
p_offset = [0; 0; 0];

x_offset = screw2dq( theta_offset, u_offset, d_offset,  cross( p_offset, u_offset ) );
 

 theta1 = q(1);
 u1 = [ 0; 0; 1 ];
 p1 =  [ 0; 0; 0.11 ];
 
 x1 = screw2dq( theta1, u1, 0,  cross( p1, u1 ) );
 
  
 theta2 = q(2);
 u2 = [ 0; -1; 0 ];
 p2 =  [ 0; 0; 0.3105 ];
 
 x2 = screw2dq( theta2, u2, 0,  cross( p2, u2 ) );
 
 
 theta3 = q(3);
 u3 = [ 0; 0; 1 ];
 p3 =  [ 0; 0; 0.51 ];
 
 x3 = screw2dq( theta3, u3, 0,  cross( p3, u3 ) );
 
 
 theta4 = q(4);
 u4 = [ 0; 1; 0 ];
 p4 =  [ 0; 0; 0.7105 ];
 
 x4 = screw2dq( theta4, u4, 0,  cross( p4, u4 ) );
 
 
 
 theta5 = q(5);
 u5 = [ 0; 0; 1 ];
 p5 =  [ 0; 0; 0.91 ];
 
 x5 = screw2dq( theta5, u5, 0,  cross( p5, u5 ) );
 
 
 
 theta6 = q(6);
 u6 = [ 0; -1; 0 ];
 p6 =  [ 0; 0; 1.105 ];
 
 x6 = screw2dq( theta6, u6, 0,  cross( p6, u6 ) );
 
 
 theta7 = q(7);
 u7 = [ 0; 0; 1 ];
 p7 =  [ 0; 0; 1.1785 ];
 
 x7 = screw2dq( theta7, u7, 0,  cross( p7, u7) );
 
 
 x01 = x1;
 x02 = muldualpq( x01, x2 );
 x03 = muldualpq( x02, x3 );
 x04 = muldualpq( x03, x4 );
 x05 = muldualpq( x04, x5 );
 x06 = muldualpq( x05, x6 );
 x07 = muldualpq( x06, x7 );
 x0e = muldualpq( x07, x_offset );
  
 
  %%%%%%%%%%%%%%%%%%%%%%%%%
 %%  Kuka robot nodes %%%%%%%%%%%%%
  p0 = [0; 0; 0];
  p1 = p1; 
 
 x = x01;
 p2_vis = muldualpq( muldualpq( x, [ 1; zeros(4,1); p2 ] ),  conjdualq(x) );
 p2 = p2_vis(6:8); 
 
 x = x02;
 p3_vis = muldualpq( muldualpq( x, [ 1; zeros(4,1); p3 ] ),  conjdualq(x) );
 p3 = p3_vis(6:8); 
  
 
 x = x03;
 p4_vis = muldualpq( muldualpq( x, [ 1; zeros(4,1); p4 ] ),  conjdualq(x) );
 p4 = p4_vis(6:8); 
  
 x = x04;
 p5_vis = muldualpq( muldualpq( x, [ 1; zeros(4,1); p5 ] ),  conjdualq(x) );
 p5 = p5_vis(6:8); 
  
 x = x05;
 p6_vis = muldualpq( muldualpq( x, [ 1; zeros(4,1); p6 ] ),  conjdualq(x) );
 p6 = p6_vis(6:8); 
  
 x = x06;
 p7_vis = muldualpq( muldualpq( x, [ 1; zeros(4,1); p7 ] ),  conjdualq(x) );
 p7 = p7_vis(6:8); 
 
 
 pe = [ 0; 0; 1.215];
 x = x07;
 pe_vis = muldualpq( muldualpq( x, [ 1; zeros(4,1); pe ] ), conjdualq(x) );
 pe = pe_vis(6:8);
  
 
 kuka_wire = [ p0, p1, p2, p3, p4, p5, p6, p7, pe ];
   
 
 
 
 
 
 
 
 
 
 
 
 