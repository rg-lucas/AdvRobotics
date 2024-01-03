function interseclines (l,m)

% Calculating and plotting the intersection
y_int = (m(2)*cos(l(1))-l(2)*cos(m(1)))/(sin(m(1))*cos(l(1))-sin(l(1))*cos(m(1)));
x_int = (l(2)-y_int*sin(l(1)))/cos(l(1));
plot(x_int, y_int, 'r.', 'markersize', 20);

% Calculating and plotting the intersection
%y_int = (m(3)*l(1)-l(3)*m(1))/(m(2)*l(1)-l(2)*m(1));
%x_int = (l(3)-y_int*l(2))/l(1);
%plot(x_int, y_int, 'r.', 'markersize', 20);


hold on; grid on;

endfunction
