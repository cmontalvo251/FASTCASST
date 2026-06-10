clear
clc
close all

global AUTOPILOT WPcounter tGPS lat_GPS lon_GPS psi_GPS GPS_speed servoAcommand deltaTcommand
global xmeasured ymeasured psimeasured umeasured

AUTOPILOT = 1;

tstart = 0.0;
tend = 500;
dt = 0.01;
tout = tstart:dt:tend;
%%%1D Simulation
%%%%%Initial conditions
servo_Angle = 93*pi/180;
throttle0 = 88*pi/180;
%%%last 4 zeros,xDELAY,yDELAY,psiDELAY,uDELAY
%%%xinitial = [x,y,psi,u,v,r,servo,throttle,delayx4]
xinitial = [0;0;0;0;0;0;servo_Angle;throttle0;0;0;0;0];

%%%Pre-allocate matrices
xout = zeros(length(xinitial),length(tout));
xout(:,1) = xinitial;
xmeasured_vec = xout(1,:)*0;
ymeasured_vec = xout(1,:)*0;
psimeasured_vec = xout(1,:)*0;
umeasured_vec = xout(1,:)*0;
servoA_command_vec = xout(1,:)*0;
deltaT_command_vec = xout(1,:)*0;

%%%Setup waypoint counter and GPS stuff
tGPS = 0;
WPcounter = 1;
lat_GPS = 0;
lon_GPS = 0;
psi_GPS = 0;
GPS_speed = 0;

%%%Make each simulation unique
RandStream('mt19937ar','seed',rand);

%%%Printing stuff
next = 10;
printnext = next;
current = 0;
for idx = 1:length(tout)-1
   zk = xout(:,idx);
   tk = tout(idx);
   k1 = Derivatives(tk,zk);
   k2 = Derivatives(tk+dt/2,zk+k1*dt/2);
   k3 = Derivatives(tk+dt/2,zk+k2*dt/2);
   k4 = Derivatives(tk+dt,zk+k3*dt);
   kphi = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
   xout(:,idx+1) = zk + kphi*dt;
   current = tk/tend*100;
   if current > printnext
       disp(['Simulation ',num2str(printnext),'% Complete'])
       printnext = printnext + next;
   end
   servoA_command_vec(idx) = servoAcommand;
   deltaT_command_vec(idx) = deltaTcommand;
   xmeasured_vec(idx) = xmeasured;
   ymeasured_vec(idx) = ymeasured;
   psimeasured_vec(idx) = psimeasured;
   umeasured_vec(idx) = umeasured;
end
servoA_command_vec(end) = servoAcommand;
deltaT_command_vec(end) = deltaTcommand;
xmeasured_vec(end) = xmeasured;
ymeasured_vec(end) = ymeasured;
umeasured_vec(end) = umeasured;
disp(['Simulation ',num2str(printnext),'% Complete'])
xout = xout';

%%%Convert tout to minutes
tout = tout/60;

%%%%PLot GPS speed
plottool(1,'Speed',12,'Time (min)','GPS Speed (m/s)');
plot(tout,xout(:,4),'b-','LineWidth',2)
plot(tout,xout(:,12),'r--','LineWidth',2)
plot(tout,umeasured_vec,'g-','LineWidth',2)
legend('Actual','Delayed Value','Measured')
title('Velocity')
%%%%Plot Steering
plottool(1,'Steering',12,'Time (min)','Steering (deg)');
plot(tout,xout(:,7)*180/pi,'b-','LineWidth',2)
plot(tout,servoA_command_vec,'r--','LineWidth',2)
title('Steering vs Time')
%%%%Plot Throttle
plottool(1,'Throttle',12,'Time (min)','Throttle (deg)');
plot(tout,xout(:,8),'b-','LineWidth',2)
plot(tout,deltaT_command_vec,'r--','LineWidth',2)
title('Throttle vs Time')
%%%PLot heading
plottool(1,'Heading (Deg)',12,'Time (min)','Heading (deg)');
plot(tout,wrap(xout(:,3))*180/pi,'b-','LineWidth',2);
psiDELAY = wrap(xout(:,11))*180/pi;
plot(tout,psiDELAY,'r--','LineWidth',2)
plot(tout,wrap(psimeasured_vec)*180/pi,'g-','LineWidth',2)
legend('Actual','Delayed Heading','Measured')
title('Heading vs Time')
%%%%Here's a lat lon plot for kicks
plottool(1,'XvsY',12,'X (m)','Y (m)');
plot(xout(:,1),xout(:,2),'b-','LineWidth',2)
plot(xout(:,9),xout(:,10),'r--','LineWidth',2)
plot(xmeasured_vec,ymeasured_vec,'g-','LineWidth',2)
legend('Actual','Delayed','Measured')
title('Vehicle Path')