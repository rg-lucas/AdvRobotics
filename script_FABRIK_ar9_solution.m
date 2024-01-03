%% FABRIK algorithm in 3D
clear; close all; clc;

%% robot in 2D or 3D space
inXD = 3;
%% a random target
tu = randn(inXD,1); %random direction
tu(3) = abs(tu(3)); %orient it toward the sky
target0 = 5*tu/norm(tu);
target = target0;

%%randomly generated line segments
n = 6; % number of segments
base = zeros(inXD,1);
u = rand(inXD,n); %random direction vectors
joint_locations(:,1) = u(:,1)/norm(u(:,1)); % 1st joint position at a unit distance
%% joint positions with unit lengths
for i = 2:n
  joint_locations(:,i) = joint_locations(:,i-1) + u(:,i)/norm(u(:,i));
end
segments = [base, joint_locations ]; % all positions (n+1)

%% plot initial configuration
figure;
for i = n:-1:1
  drawLineSegment3D( segments(:,i+1), segments(:,i) );
end
plot3(target0(1), target0(2), target0(3), 'go', 'markersize', 30, 'linewidth', 10);
axis equal; grid on; axis([-6 6 -6 6 -0.5 6 ]);
title('Initial configuration.', 'fontsize', 20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% FABRIK LOOP %%%%%%%%%%%%%%%%%%%%%%%%
eps = 0.01;
maxIter = 3;
k = 0;

do
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%% FORWARD REACH (loop from tip to base)%%%
  for i = n+1:-1:2

    head = segments(:,i);
    tail = segments(:,i-1);

    [new_head, new_tail] = reach( head, tail, target); %reach
    segments(:,i) = new_head; % segment's head moves to the target
    target = new_tail; % target moves to the new tail position

  end
  %update the last segment's tail (base moves!)
  segments(:,1) = new_tail;

  % backward reach (loop from base to tip)
  target = base;
  for i = 1:n
    head = segments(:,i);
    tail = segments(:,i+1);

    [new_head, new_tail] = reach( head, tail, target); %reach
    segments(:,i) = new_head; % segment's head moves to the target
    target = new_tail; % target moves to the new tail position
  end
  % update the last segment's tail (tip moves!)
  segments(:, n +1) = new_tail;

  tip = segments(:, n+1);
  k = k +1;
  disp(['norm(target0 - tip) ', num2str(norm(target0 - tip)), ', k = ', num2str(k)]);

until( norm(target - tip) < eps || k >maxIter)
%%%%%%%%% END of FABRIK LOOP %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%plot FABRIK solution
figure;
for i = n:-1:1
  drawLineSegment3D(segments(:, i+1), segments(:,i) );
end
plot3(target0(1), target0(2), target0(3), 'go', 'markersize', 30, 'linewidth', 10);
axis equal; grid on; axis([-6 6 -6 6 -0.5 6 ]);
title('FABRIK.', 'fontsize', 20);
