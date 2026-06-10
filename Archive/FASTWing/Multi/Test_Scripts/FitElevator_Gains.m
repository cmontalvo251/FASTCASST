close all
clear
clc


kpe = [-0.05 -0.15 -0.18 -0.22 -0.25]';
kde = [-0.25 -0.25 -0.35 -0.55 -0.75]';

NAC = [1:5]';

figure()
plot(NAC,kpe,'b*')

astar = polyfit(NAC([1 5]),kpe([1 5]),1)
kptilde = polyval(astar,NAC);

hold on

plot(NAC,kptilde,'r--')


figure()
plot(NAC,kde,'b*')

H = [ones(length(kde),1),NAC,NAC.^2];
astar = inv(H'*H)*H'*kde
kdtilde = H*astar;
hold on

plot(NAC,kdtilde,'r--')



