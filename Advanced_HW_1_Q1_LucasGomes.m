%Question 1 - Advanced Robotics - Homework 1
%Lucas Resende Gomes

close all;
clc;
clear;

k = [1,1,1];

k_hat = point3DEuclidean2DQ( k );

x = [2, 2, 2];
x_hat = point3DEuclidean2DQ(x)

xhat_2stars = [  quartenionConjugateforQ(x_hat(1:4));
                -quartenionConjugateforQ(x_hat(5:8));



%transformation

z_hat = dqprod (x_hat , k_hat);
k_hat_moved = dqprod(z_hat , x_hat_star)

%plot of the original point
figure;
plotPointDQ(k_hat);
grid on;

%plot of the final point
figure;
plotPointDQ( k_hat_moved);
grid on;

