clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Camera Pose at A wrt to the fixed frame R
% s-pace
u_AR = [ 1; 0; 0];          % u (orientation axis)
theta_AR = -60*deg2rad;     % theta (orientation angle)
t_AR = [ -0.1; -0.1; 0.9 ]; % position

% dq-space
dq_AR = uthetat2dq( u_AR, theta_AR, t_AR );  % dual quaternion

% C-space
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );
X_AR = [ R_AR, t_AR ; 0 0 0 1];  % Cartesian space pose


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Camera Pose at B wrt to the fixed frame R
% s-pace
u_BR = [ 1; 0; 0];         % u (orientation axis)
theta_BR = -90*deg2rad;    % theta (orientation angle)
t_BR = [ 0.3; 0.2; 0.8 ];  % position

% dq-space
dq_BR = uthetat2dq( u_BR, theta_BR, t_BR );  % dual quaternion

% C-space
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );
X_BR = [ R_BR, t_BR ; 0 0 0 1];  % Cartesian space pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
traces = []; % the trajectory of the camera frame
%traces_error = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 0;  % current time
tf = 0.6; % final time
dt = 0.01; % control sampling time
figure;


% Initialize an empty vector
%resultVector = zeros(8, 60);
%i=1;






% Initialize arrays to store errors and time
errors = [];
time_array = [];







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while(time < tf)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%VOTRE CODE ICI
%% Compute error : you can use dual quaternion functions first
error_dq_AB = muldualpq( conjdualqsimple( dq_BR ), dq_AR);
[ u_AB, theta_AB, R_AB, t_AB] = dualq2uthetaRt (error_dq_AB);

error_AB = [ R_AB, t_AB ; 0 0 0 1];
%% Control Law
k = 10;
v = -k * (R_AB)' * t_AB;
w = -k * theta_AB * u_AB;
C_L_AB = [ v; w];

%% Convert Control Law in the good frame
T_BR = [ R_BR, skew(t_BR) * R_BR;
            zeros(3 ,3), R_BR ];
C_L_AR = T_BR * C_L_AB;
%% Move Camera
v = C_L_AR(1:3);
w = C_L_AR(4:6);
theta = norm(w);
if ( theta == 0) u= [0;0;1]; else u = w/ norm(w); end
update_dq_AR = uthetat2dq(u, dt*theta , dt*v );
dq_AR = muldualpq (update_dq_AR, dq_AR);

[u_AR, theta_AR, R_AR, t_AR]= dualq2uthetaRt (dq_AR);
X_AR = [R_AR, t_AR; 0 0 0 1];
% Calculate the error in dual quaternion form
error_dq_AB = muldualpq( conjdualqsimple( dq_BR ), dq_AR);

% Extract error components for analysis
[u_error, theta_error, R_error, t_error] = dualq2uthetaRt(error_dq_AB);

% Append error magnitude to the errors array
errors = [errors, norm([t_error; theta_error])];
time_array = [time_array, time];

%FIN DE VOTRE CODE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Scene
time = time + dt;
traces = [ traces, t_AR ]; % the trajectory of the camera frame
%traces_error=[ traces_error,t_AB]; % the error

clf; hold on;
plot3( traces(1,:), traces(2,:), traces(3,:), 'k' );
%plot3( traces_error(1,:), traces_error(2,:), traces_error(3,:), 'r' );
plot_pose( X_AR, 'k'  );  plot_camera( X_AR, 'b' );
plot_pose( X_BR, 'k'  );  plot_camera( X_BR, 'r' );
%plot_pose( error_AB, 'r'  );
axis([-1.2 1.2 -1.2 1.2 -1.4 1.4]);
view( 64, 34 );
title( num2str(time) );
drawnow;

end

figure;
plot(time_array, errors);
xlabel('Time (s)');
ylabel('Error Magnitude');
title('Error in Camera Pose vs. Time');
grid on;
%T = linspace(time,tf,60);
%plot(errorVector,T);
