% rotation about the origin
clear; close all;
p = [1;0]; % point in Cartesian coordinates

p1=[1;-2];
p2=[1;2];
n=10;



%phi = deg2rad(0);
%d=1;  % hessian coordinates of a line

%L=[cos(phi);sin(phi);-d];% homogeneous coordinetes of a line
%nL=normalize_homogeneous_line(L);
%plot2Dline( nL , [-2,2],'k');hold on ; grid on ;% plot the line in black



theta = linspace( 0, 2*pi );

n = 10; % length of the previous trajectory to show
figure; % open a figure to use in the animation
for i=1:length(theta)
q = rot2( theta(i) )*p; % rotate the point about the origin



clf; % clear the figure

plot( q(1), q(2),'ro', 'markersize', 10, 'markerfacecolor', 'r');
axis([-3 3 -3 3]); grid on; hold on;

    t=p1-p;
    z=p2-p;
    %q1=(rot2( theta(i) )*t)+q;
    %q2=(rot2( theta(i) )*z)+q;
    q1=t+q;
    q2=z+q;

    plot([q1(1),q2(1)],[q1(2),q2(2)],'linewidth',2,'k');
    hold on;grid on;




pause(0.1); % waits 0.1 seconds
drawnow; % draw everything now
endfor
