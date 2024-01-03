% forward reach
clear; close all; clc;

% a random target
target0 = rand(2,1) + 0.5*ones(2,1);
target = target0;

% randomly generated line segments
n = 2; % number of segments
base = zeros(2,1);
joint_locations = rand(2,n);
segments = [base, joint_locations ]; % all positions (n+1)

% plot initial configuration
figure;
for i = n:-1:1
  drawLineSegment2D(segments(:,i+1), segments(:,i) );
end
plot(target0(1),target0(2), 'go', 'markerfacecolor', 'g', 'markersize', 40);
axis equal; grid on;
axis([-0.5 1.5 -0.5 1.5]);
title('Initial configuration.', 'fontsize', 20);

% forward reach (loop from tip to base)
for i = n+i:-1:2

  head = segments(:,i);
  tail = segments(:,i-1);

  [new_head, new_tail] = reach( head, tail, target); % reach
  segments(:,i) = new_head; % segment's head moves to the target
  target = new_tail; % target moves to the new tail position

end
% update the las segment's tail (base moves!)
segments(:,1) = new_tail;

% plot forward reach
figure;
for i = n:-1:1
  drawLineSegment2D( segments(:,i+1), segments(:,i) );
end
plot(target0(1),target0(2), 'go', 'markerfacecolor', 'g', 'markersize', 40);
axis equal; grid on;
axis([-0.5 1.5 -0.5 1.5]);
title('Forward reach.', 'fontsize', 20);

% backward reach (loop from base to tip)
target = base;
for i = 1:n
    head = segments(:,i);
    tail = segments(:,i+1);

    [new_head, new_tail] = reach(head, tail, target); %reach
    segments(:,i) = new_head; % segment's head moves to the target
    target = new_tail; % target moves to the new tail position
end
% update the last segment's tail (tip moves)
segments(:,n+1) = new_tail;

% plot backward reach
figure;
for i = n:-1:1
    drawLineSegment2D( segments(:,i+1), segments(:,i) );
end
plot(target0(1),target0(2), 'go', 'markersize', 40, 'linewidth', 10);
axis equal; grid on; axis([-.5 1.5 -.5 1.5]);
title('Backward reach.','fontsize', 20);
