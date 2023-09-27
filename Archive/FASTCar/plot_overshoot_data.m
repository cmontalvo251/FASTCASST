function plot_overshoot_data()

clc
close all

%%Open File
data = dlmread('GPS_Files/ECXA_2_3/ECXA_86.TXT');

%%%Grab time
time = data(:,7);
%%Truncate between 125 and 138 seconds
ts = 130;
te = 148;
s=find(time > ts,1);
e=find(time > te,1);
time = data(s:e,7);
%%Get heading
gps_heading = data(s:e,20);
%%Plot it
plot(time,gps_heading,'b*')
xlim([ts,te])

%%%Simulate using ode45
tspan = [ts,te];
[tout,xout] = ode45(@Heading,tspan,[0;0]);

heading_sim = xout(:,1)+gps_heading(1); 

%%%Plot simulation
hold on
plot(tout,heading_sim,'r-')

legend('GPS Data','Simulation')
xlabel('Time (sec)')
ylabel('Heading Angle (deg)')


function dxdt = Heading(t,x)

heading = x(1);
yaw_rate = x(2);

%%Since f is hit with w the steady state output is 1 so we
%just need to scale f by 160 in this case.
f = -160; 

%%%First we need to figure out the period
%%%It looks like 1 cycle is 7 seconds?
T = 7;
fhz = 1/T;
wd = 2*pi*fhz; 

%%%Then you just need to figure out the decrement value but it's
%probably better just to use the settling time instead
%So settling time is about 28 seconds so Ts = 4/(zed*wn)
%Problem is this formula uses wn and we just have wd? Well.
% wd = wn*sqrt(1-zed^2)
% wn = wd/(sqrt(1-zed^2)
% Ts = 4*sqrt(1-zed^2)/zed*wd
% Ts^2 = 16*(1-zed^2)/zed^2*wd^2
% Ts^2 * zed^2 * wd^2 = 16 - 16*zed^2
% (Ts^2 * wd^2 + 16) * zed^2 = 16
% zed = sqrt(16/(Ts^2*wd^2+16)
Ts = 50; %%%Turns out the settling time is more like 50 seconds you
         %just can't see it from the graph
zed = sqrt(16/(Ts^2*wd^2+16));

%%And then of course wn
wn = wd/(sqrt(1-zed^2));

%%%Use a second order model with a step forcing function
yaw_acceleration = -2*zed*wn*yaw_rate - wn^2*heading + wn^2*f; 

dxdt = [yaw_rate;yaw_acceleration];
