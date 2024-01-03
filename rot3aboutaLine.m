function H = rot3aboutaLine( theta, eL )

 u = eL(1:3); % unit direction axis
 m = eL(4:6); % moment
 
 R = rodrigues(u, theta); %3x3 rotation matrix
 
 H = [ R, (eye(3) - R)*(cross(u,m)); zeros(1,3), 1];

end
