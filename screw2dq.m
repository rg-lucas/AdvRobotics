function x = screw2dq( theta, axe, d, momen )
% theta : rotation around screw axis
% axe : screw axis
% d : translation along the screw axis
% momen : moment on the screw axis w.r.t. the fixed origin ("a point on screw axis" x "screw axis direction")  
		
q_rot = [ cos( theta/2 ); sin( theta/2 )*axe   ];

q_tr = [ (-d/2)*sin( theta/2 ); (d/2)*cos( theta/2 )*axe + sin( theta/2 )*momen  ];

x = [ q_rot; q_tr ];
	
	