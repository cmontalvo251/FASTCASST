clear
clc
close all

load ThrustvsPWMenglish.txt
x = ThrustvsPWMenglish(:,1);
y = ThrustvsPWMenglish(:,2);

figure
plot(x,y,"*")
title("Thrust vs PWM Signal");
xlabel("PWM Signal, us");
ylabel("Thrust, lbf");
grid on;

Coeff = polyfit(x,y,2)
X = linspace(0,2000,1000);
Y = polyval(Coeff,X);

figure
plot(x,y,"*")
hold on
plot(X,Y,":",'LineWidth',4)
title("Thrust vs PWM Signal");
xlabel("PWM Signal, us");
ylabel("Thrust, lbf");
xlim([1100 1900])
ylim([0 max(y)])
grid on;
h = legend('Experimental Data','Parabolic Fit');
legend (h, "location", "northwest");
