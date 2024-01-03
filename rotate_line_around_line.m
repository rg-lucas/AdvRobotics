% Definition of Line 0:

a = [0; 0; -0.5 ];
b = [0; 0; 0.5 ];
L0 = pluckerLinefrom2points(a,b);
eL = EucNormalizationPluckerLine(L0);


% Definition of Line 1:

a = [0.5; 0; 0 ];
b = [1; 0; 0 ];
L1 = pluckerLinefrom2points(a,b);



theta = linspace (0,2*pi-2*pi/30,30);


% plot the scene



for i=1:length(theta)
    % rotation transformation about a line with an angle
    H = rot3aboutaLine(theta(i), eL);
    R = H(1:3,1:3); t=H(1:3,4);

    H_bar = [       R,      zeros(3,3);
                skew(t)*R,      R       ];

    % rotate the point about the line
    L_prime = H_bar*L1;

    clf; hold on; grid on;% clear the figure
    figure;
    plotPluckerLine( eL, 3, 'b'); % line of rotation
    hold on; grid on; axis([ -2 2 -2 2 -2 2]); axis equal;
    plotPluckerLine( L_prime, 1, 'r' );
    box on;
    drawnow;
    pause(1);

endfor




plotPluckerLine( L1, 1, 'k'); % initial line
plotPluckerLine( L_prime, 1, 'r' ); % final line
