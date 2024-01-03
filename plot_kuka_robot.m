function plot_kuka_robot( q, show )
% show : (1 or 0) shows on/off the joint values on the screen 

[x0e, nodes] = fkm_visual( q );	

p0 = nodes(:,1);
p1 = nodes(:,2);
p2 = nodes(:,3);
p3 = nodes(:,4);
p4 = nodes(:,5);
p5 = nodes(:,6);
p6 = nodes(:,7);
p7 = nodes(:,8);
pe = nodes(:,9);


 
 hold on;
 plot3( nodes(1,:), nodes(2,:), nodes(3,:), 'r-', 'LineWidth', 8  ); 
 plot3( nodes(1,2:8), nodes(2,2:8), nodes(3,2:8), 'ko',  'MarkerSize', 10,   'MarkerEdgeColor', [0.6,0.6,0.6], 'MarkerFaceColor', [0.6,0.6,0.6] );
 plot3( p0(1), p0(2), p0(3), 'bs',  'MarkerSize', 15, 'MarkerFaceColor', 'b' );
 plot3( pe(1), pe(2),  pe(3), 'bs' , 'MarkerSize', 4, 'MarkerFaceColor', 'b' );
  axis equal;
 
 xoff = 0.03;
 zoff = 0.03;
 rad2deg = 180/pi;
 
 if(show)
 text(  p1(1) + xoff, p1(2), p1(3) + zoff, num2str(  q(1)*rad2deg  ) ); 
 text(  p2(1) + xoff, p2(2), p2(3) + zoff, num2str(  q(2)*rad2deg  ) ); 
 text( p3(1) + xoff, p3(2), p3(3) + zoff,  num2str(  q(3)*rad2deg   ) ); 
 text(  p4(1) + xoff, p4(2), p4(3) + zoff,  num2str(  q(4)*rad2deg   ) ); 
 text(  p5(1) + xoff, p5(2), p5(3) + zoff,  num2str(  q(5)*rad2deg   ) ); 
 text(  p6(1) + xoff, p6(2), p6(3) + zoff, num2str(  q(6)*rad2deg   ) ); 
 text(  p7(1) + xoff, p7(2), p7(3) + zoff, num2str(  q(7)*rad2deg   ) ); 
 end
 
 