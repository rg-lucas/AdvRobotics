clear all; close all;
a=[4;3];
r=1;
p1=[5;3];
p2=[7;3];
angle=linspace(0,2*pi,100);
theta=linspace(0, 2*pi)
n=10;
figure;

for i=1:length(theta)

    t=p1-a;
    z=p2-a;
    q1=(rot2( theta(i) )*t)+a;
    q2=(rot2( theta(i) )*z)+a;

    clf;

    plot([q1(1),q2(1)],[q1(2),q2(2)],'linewidth',2,'k');
    hold on;grid on;

    % draw circle's center
    plot(a(1),a(2),'bo','markersize',7,'markerfacecolor','r');

    % draw the circle
    X= r * cos(angle) + a(1);
    Y= r * sin(angle) + a(2);
    plot(X, Y ,'b','linewidth',1);

    axis([ 0 8 0 7]);

    pause(0.1);
    drawnow;

endfor

