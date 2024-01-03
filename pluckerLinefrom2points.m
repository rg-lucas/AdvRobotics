function L = pluckerLinefrom2points( x, y )

Lh = y - x;
Lo = cross(x,y);
L = [ Lh; Lo ];

end