%Question 1 - Advanced Robotics - Homework 1
%Lucas Resende Gomes

close all; clc; clear;

%initial point coordinates
q = [0; 1; 1]
pointDQ = point3DEuclidean2DQ(q)

%final point coordinates
x = [3; 2; 3]

x_hat = dq_from_tutheta(x, x, 3)

x_hat_2stars = [  quartenionConjugateforQ(x_hat(1:4));
                  -quartenionConjugateforQ(x_hat(5:8))        ]

pointDQlineB = dqprod(x_hat, pointDQ)
pointDQline = dqprod(pointDQlineB, x_hat_2stars)

figure
plotPointDQ(pointDQ)
hold on; grid on;

figure
plotPointDQ(pointDQlineB)
hold on; grid on;
