%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Random Sample Consensus (RASNAC) for 2D line fitting %%
clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate 2D points for ata set S %%%%%%%%%%%%%%%%%%%%%%
m=10; %the number of points in the data set S
p=[rand(2,1);1]; % a random 2D point in homogeneous coordinates
q=[rand(2,1);1]; % a random 2D point in homogeneous coordinates
% Basic Robotics - Lecture#3
line = cross(p,q); % homogeneous line equations as ax+by+c=0 where line = [a; b; c]
a = line(1); b = line(2); c = line(3);

x = linspace(0,1,m);
y = -(a/b)*x - c/b;
S = [x; y]; % the data set

S = S + 0.1*rand(2,m); % add small noise
%make some outliers
S(1,2) = S(1,2)+1;
S(1,5) = S(1,5)-1;
S(2,8) = S(2,8)-1;

%% show the data set S
figure; plot(S(1,:),S(2,:),'ko','markersize',10,'markerfacecolor','k')
title('Data set','fontsize',20);
axis([-3 4 -3 4]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Random sample selection %%%%%%%%%%%%%%%%%%%%%%%%%%
randim(1) = randi(m);
randim(2) = randi(m);

while(randim(1) == randim(2))

  s = 2; %sample size for a line
  for i=1:s
    randim(1) = randi(m);
  endfor

endwhile

if (randim(1) != randim(2))
  selection(:,1) = S(:,randim(1));
  selection(:,2) = S(:,randim(2));
endif

%% show the selection
figure; plot(S(1,:),S(2,:),'ko','markersize',10,'markerfacecolor','k'); hold on;
plot(selection(1,:), selection(2,:),'ro','markersize',10,'markerfacecolor','r');
title('Random sample subset','fontsize', 20);
axis([-3 4 -3 4]);

%% instantiate the model from this sample subset
p = [ selection(:,1); 1]; % put into homogeneous coordinates the selection point
q = [ selection(:,2); 1]; % put into homogeneous coordinates the selection point
line_model = cross(p,q);
%% show the model
figure; plot(S(1,:),S(2,:),'ko','markersize',10,'markerfacecolor','k'); hold on;
plot(selection(1,:), selection(2,:),'ro','markersize',10,'markerfacecolor','r');
nL = normalize_homogeneous_line(line_model);
plot2Dline( nL, [-2, 3], 'b');
title('instantiate the model', 'fontsize', 20);
axis([-3 4 -3 4]);





