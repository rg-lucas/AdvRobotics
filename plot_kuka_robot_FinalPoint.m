function plot_kuka_robot( q_d)
% show : (1 or 0) shows on/off the joint values on the screen

[x0e, nodes] = fkm_visual( q_d );

p0 = nodes(:,1);
p1 = nodes(:,2);
p2 = nodes(:,3);
p3 = nodes(:,4);
p4 = nodes(:,5);
p5 = nodes(:,6);
p6 = nodes(:,7);
p7 = nodes(:,8);
pe_final = nodes(:,9);

