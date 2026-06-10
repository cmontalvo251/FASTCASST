% clc
clear
close all
format compact
%% Initial Values %%
alpha = 0*pi/180;
trm = [0 0 -200 60*pi/180 alpha 0 16 0 16*atan(alpha) 0 0 0]'; %[x y z phi theta psi u v w p q r] 
nCrafts = 1;
%% Controller %%
controller = 1; % 0 = Open Loop, 1 = PID,
%% Simulate %%
%%% Run RK4 with Derivatives.m routine
dt = .01;
time = 0:dt:25;
xinitial = trm;
xstate = SixDOF(xinitial,time,controller,nCrafts);

%% Plot %%
%-------------------------------------------------------------------------
% figure
% subplot(3,1,1)
% plot(time,xstate(1,:));xlabel('Time(s)');ylabel('x')
% subplot(3,1,2)
% plot(time,xstate(2,:));ylabel('y')
% subplot(3,1,3)
% plot(time,-xstate(3,:));ylabel('z')
% 
% figure
% subplot(3,1,1)
% plot(time,xstate(4,:)*180/pi);xlabel('Time(s)');ylabel('\phi')
% subplot(3,1,2)
% plot(time,xstate(5,:)*180/pi);ylabel('\theta')
% subplot(3,1,3)
% plot(time,xstate(6,:)*180/pi);ylabel('\psi')
% 
% figure
% subplot(3,1,1)
% plot(time,xstate(7,:));xlabel('Time(s)');ylabel('u')
% subplot(3,1,2)
% plot(time,xstate(8,:));ylabel('v')
% subplot(3,1,3)
% plot(time,-xstate(9,:));ylabel('w')
%------------------------------------------------------------------------ 
% figure
% subplot(3,1,1)
% plot(time,xstate(10,:));xlabel('Time(s)');ylabel('p')
% subplot(3,1,2)
% plot(time,xstate(11,:));ylabel('q')
% subplot(3,1,3)
% plot(time,-xstate(12,:));ylabel('r')

% figure()
% plot(time,xstate(7,:));xlabel('time');ylabel('u');
% figure()
% plot(time,xstate(9,:));xlabel('time');ylabel('w');

% figure()
% plot(xstate(5,:),xstate(11,:));xlabel('\theta');ylabel('q')
% hold on
% plot(xstate(5,1),xstate(11,1),'gs')
% plot(xstate(5,end),xstate(11,end),'r^')
% 
% figure()
% plot(xstate(7,:),xstate(9,:));xlabel('u');ylabel('w')
% hold on
% plot(xstate(7,1),xstate(9,1),'gs')
% plot(xstate(7,end),xstate(9,end),'r^')