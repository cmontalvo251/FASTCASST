function Throttle_Controller()
global steady_state_error err F

clc
close all

dt = 0.01;
tout = 0:dt:3;
F_vec = 0*tout;
err_vec = 0*tout;

N = 3;

xinitial = [0;0;0];

xout = zeros(N,length(tout));

xout(:,1) = xinitial;

for idx = 1:length(tout)-1
  tk = tout(idx);
  xk = xout(:,idx);
  k1 = Derivatives(tk,xk);
  k2 = Derivatives(tk+dt/2,xk+k1*dt/2);
  k3 = Derivatives(tk+dt/2,xk+k2*dt/2);
  k4 = Derivatives(tk+dt,xk+k3*dt);
  xRK4 = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
  xout(:,idx+1) = xk + xRK4*dt;
  if F < 0
     xout(1:2,idx+1) = 0;
  end
  %%%Integrate Error Arduino Style
  xout(3,idx+1) = xk(3) + 0.01*err;
  F_vec(:,idx+1) = F;
  err_vec(:,idx+1) = err;
end

h1 = figure();
plot(tout,xout(1,:),'LineWidth',2)
hold on
plot(tout,xout(2,:),'c-','LineWidth',2)
plot(tout,xout(3,:),'k-','LineWidth',2)
set(gca,"fontsize",18)
xlabel('Time (sec)')
ylabel('V (m/s)')
grid on
plot(tout,F_vec,'r-','LineWidth',2)
plot(tout,err_vec,'g-','LineWidth',2)
legend('Velocity','Velocity Measured','Integral','Force','Error')


disp(['Steady State Error = ',num2str(steady_state_error)])

function dxdt = Derivatives(tin,xin)
global steady_state_error err F

vel = xin(1);

%F = 1; %%Step input

%No measuring dynamics
%vel_measured = vel;
%Assume Dynamics are Measured Now
vel_measured = xin(2);

kint = xin(3);

%Unity Feedback
vel_command = 2;
kp = 4.8;
ki = 1;
err = vel_command - vel_measured;
F = kp*err + ki*kint;

%%%Integrate error
kintdot = 0*err;

%%%Saturation Block
if F > 10
F = 10;
end

%%%Compute STeady State Error with proportional feedback
steady_state_error = vel_command - kp/(kp+1)*vel_command;

%%%Platn Dynamics
Gtau = 4;
veldot = Gtau*(F-vel);

%%%Measurement Dynamics
Htau = 2;
velmdot = Htau*(vel-vel_measured);

dxdt = [veldot;velmdot;kintdot];