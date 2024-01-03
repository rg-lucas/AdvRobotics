%Question 2 - Advanced Robotics - Homework 1
%Lucas Resende Gomes

close all;
clc;
clear;

q = [2; 2; 2];
%q = q/norm(q);
u = [4; 4; 4];
u = u/norm(u);

L = qU2pluckerLine(q, u)
L_hat = linePlucker2DQ(L)

x = [5; 1; 1]
x_hat = dq_from_tutheta(x, x, 3)

x_hat_star = [  quartenionConjugateforQ(x_hat(1:4));
                -quartenionConjugateforQ(x_hat(5:8))        ]

L_hat_lineB = dqprod(x_hat, L_hat)
L_hat_line = dqprod(L_hat_lineB, x_hat_star)

figure
pointDQ = point3DEuclidean2DQ(q)
plotLineDQatApoint(L_hat, pointDQ, 1, 'r')
hold on; grid on;

figure
plotLineDQatApoint(L_hat_line, pointDQ, 1, 'b')
hold on; grid on;

