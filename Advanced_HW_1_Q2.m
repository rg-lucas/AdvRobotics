%Question 2 - Advanced Robotics - Homework 1
%Lucas Resende Gomes

close all;
clc;
clear;

%initial cocrdinates
q = [2; 2; 2];
u = [4; 4; 4];
u = u/norm(u);

L = qU2pluckerLine(q, u)
L_hat = linePlucker2DQ(L)

%final coordinates
x = [5; 1; 1]
x_hat = dq_from_tutheta(x, x, 4)

x_hat_star = [  quartenionConjugateforQ(x_hat(1:4));
                -quartenionConjugateforQ(x_hat(5:8))        ]

L_hat_lineB = dqprod(x_hat, L_hat)
L_hat_line = dqprod(L_hat_lineB, x_hat_star)


%initial
figure
pointDQ = point3DEuclidean2DQ(q)
plotLineDQatApoint(L_hat, pointDQ, 1, 'r')
title('initial position', 'fontsize', 20)
hold on; grid on;

%final
figure
plotLineDQatApoint(L_hat_line, pointDQ, 1, 'b')
title('final position - after transfomation', 'fontsize', 20)
hold on; grid on;

