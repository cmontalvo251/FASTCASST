clear
clc
close all

global throttle_deg steering_deg total_time_arduino_and_GPS_min deltaT AUTOPILOT

%%%Import Data
%data = dlmread('ECXA_9_30/ECXA_00_Parking_Lot.TXT');
%data = dlmread('GPS_Files/11_21_16/ECXA_61.TXT');
%data = dlmread('GPS_Files/12_15_2016/ECXA_53.TXT');
%data = dlmread('GPS_Files/ECXA_1_10/ECXA_72.TXT');
%data = dlmread('GPS_Files/1_13_2017/ECXA_78.TXT');
data = dlmread('GPS_Files/ECXA_2_3/ECXA_86.TXT'); %%%86 and 89
%%%86 is high gain which is cool cuz the overshoot is pretty cool - kp=16?
%%%89 is low gain and there is some steady state error - kp = 8?
%%%Maybe next time we should try kp = 10 or 12?

%AUTOPILOT = 0;

%%%Columns of Data are in the following format
%GPS.year - 1
%GPS.month - 2
%GPS.day - 3
%GPS.hour - 4 - GPSstart_col
%GPS.minute - 5
%GPS.seconds - 6
%ArduinoTime - 7
%GPS.fix - 8 
%GPS.lat - 9
%GPS.long - 10
%trainer switch - 11
%autopilot 
%umeasured(m/s) 
%receiver_pulse_throtte 
%deltaT 
%x
%y 
%xcommand 
%ycommand 
%GPS.angle(deg) 
%psic(deg) 
%dpsi(deg)
%receiver_pulse_servo 
%servoA 

%%%%%Remove all "BAD" GPS data
data = remove_bad_gps(data,8,1,'ECXA');

%%%Plot only a few rows
%data = data(100:500,:);

%%%%Combine Arduino and GPS time data
GPS_start_col = 4;
total_time_arduino_and_GPS_min = combine_GPS_arduino_time(data,GPS_start_col,1);

Names={'Year','Month','Day','Hour','Minute','Seconds','ArduinoTime',...
    'Fix','Latitude (Deg)','Longitude (deg)','Trainer Switch','Autopilot',...
    'Umeasured(m/s)','Receiver Pulse Throttle (ms)','Throttle (deg)'...
    'X (m)','Y (m)','Xcommand (m)','Y command (m)','GPS Compass (deg)'...
    'Psi Command (deg)','Delta Psi (deg)','Receiver Pulse Servo','Servo (deg)'};
for idx = 1:length(Names)
    plottool(1,Names{idx},12,'Time (min)',Names{idx});
    %%%Plot this if you want to plot absolute arduino time plus GPS time in min
    plot(total_time_arduino_and_GPS_min,data(:,idx),'LineWidth',2);
    if idx == 16 || idx == 17
        ylim([-100 100])
    end
    
    %%%Plot this if you want to plot absolute arduino time plus GPS time in hrs
    %plot(total_time_arduino_and_GPS_hr,data(:,4+idx),'LineWidth',2)
% 
%   %%%Plot this if GPS time screwed up and you just want arduino time
%   %plot(time_arduino_min,data(:,4+idx),'LineWidth',2)
end

plottool(1,'Psi',12,'Time (min)','Psi (deg)')
psic = data(:,21);
psi = data(:,20);
plot(total_time_arduino_and_GPS_min,psic,'r-')
plot(total_time_arduino_and_GPS_min,psi,'b-')
legend('Command','Measured')

plottool(1,'Dpsi',12,'Time (min)','Dpsi (rad)')
dpsi = pi/180*data(:,22);
psi = pi/180*psi;
psic = pi/180*psic;
spsi = sin(psi);
cpsi = cos(psi);
spsic = sin(psic);
cpsic = cos(psic);
dpsi_computed = atan2(spsi.*cpsic-cpsi.*spsic,cpsi.*cpsic+spsi.*spsic);
plot(total_time_arduino_and_GPS_min,dpsi,'b-')
plot(total_time_arduino_and_GPS_min,dpsi_computed,'r-')
legend('Arduino','MATLAB')

plottool(1,'Servo',12,'Time (min)','Servo')
servo = data(:,24);
STEER_STRAIGHT = 103; %%%103 or 93
servo_computed = -16*dpsi_computed + STEER_STRAIGHT; %%Kp is -16 or -8
plot(total_time_arduino_and_GPS_min,servo,'b-')
plot(total_time_arduino_and_GPS_min,servo_computed,'r-')
plot(total_time_arduino_and_GPS_min,ones(length(servo),1)*STEER_STRAIGHT,'k-')
ylim([0 180])

plottool(1,'LatvsLon',12,'Longitude (deg)','Latitude (deg)')
latitude = data(:,9);
longitude = data(:,10);
autopilot = data(:,12);
plot(longitude,latitude,'LineWidth',2)
plot(longitude(autopilot==1),latitude(autopilot==1),'r-','LineWidth',2)
legend('Autopilot Off','Autopilot On')

plottool(1,'XvsY',12,'Y (m)','X (m)');
origin  = [30.706606, -88.159402]; %%Muni from Arduino
%origin = [latitude(1),longitude(1)];
[x,y] = convertLATLON(latitude,longitude,origin);
plot(y(1),x(1),'bs','MarkerSize',10)
plot(y,x,'LineWidth',2)
plot(y(autopilot==1),x(autopilot==1),'r-','LineWidth',2)
ax = axis; %%Save axis
x_ard = data(:,16);
y_ard = data(:,17);
plot(y_ard,x_ard,'g--','LineWidth',2)
axis(ax) %%reset axis
legend('Start','XY Computed','XY Autopilot','XY Arduino')

%save_plots()

break

%%%Throttle Debug
plottool(1,'Throttle Debug',12,'Time (min)','Umeasured (m/s)');
plot(total_time_arduino_and_GPS_min,data(:,11))
plot(total_time_arduino_and_GPS_min,data(:,12),'r-')

plottool(1,'Throttle Debug',12,'Time (min)','Receiver Signal');
plot(total_time_arduino_and_GPS_min,data(:,11))
plot(total_time_arduino_and_GPS_min,data(:,13)-1300,'r-')

plottool(1,'Throttle Debug',12,'Time (min)','Throttle (deg)');
plot(total_time_arduino_and_GPS_min,data(:,11))
plot(total_time_arduino_and_GPS_min,data(:,14),'r-')


break

%%%%STRIP OUT CERTAIN TIMES
%%%Turning 2.2 to 2.7 this is for Parking Lot data
%%%Driving Straight 0.67 to 0.80
tstart = 0.67;
tend = 0.8;
lstart = find(total_time_arduino_and_GPS_min > tstart,1);
if tend > total_time_arduino_and_GPS_min
    lend = length(total_time_arduino_and_GPS_min);
else
    lend = find(total_time_arduino_and_GPS_min > tend,1);
end

data = data(lstart:lend,:);
total_time_arduino_and_GPS_min = total_time_arduino_and_GPS_min(lstart:lend);

[total_time_arduino_and_GPS_min,I,J] = unique(total_time_arduino_and_GPS_min);
data = data(I,:);

%%%%Now let's just loop through the rest of the file and plot everything
%%%%Here are the columns in case you don't want to scroll up
%%% 1=GPS.hour 2=GPS.minute 3=GPS.seconds 4=ArduinoTime 
%%% 5=GPS.fix 6=GPS.lat 7=GPS.long 8=GPS.speed(knots) 
%%% 9=GPS.angle(deg) 10=steering_pulse 11=steering_deg
%%% 12=throttle_pulse 13=throttle_deg

%%%Extract Usefule Data
throttle_deg = data(:,13);
steering_deg = data(:,11);
GPS_Speed = data(:,8)*.51444444;
GPS_heading = data(:,9);
del_max = max(steering_deg)
del_min = min(steering_deg)
T_max = max(throttle_deg)
T_min = min(throttle_deg)
umax = max(GPS_Speed)

%%%1D Simulation
xinitial = [0;0;GPS_heading(1)*pi/180;GPS_Speed(1);0;0;steering_deg(1)...
    *pi/180;throttle_deg(1)];
tsim = linspace(tstart*60,tend*60,1000);

[tout,xout] = ode45(@Derivatives,tsim,xinitial);   

%%%Convert tout to minutes
tout = tout/60;

%%%%Plot Throttle
plottool(1,'Throttle',12,'Time (min)','Throttle (deg)');
plot(total_time_arduino_and_GPS_min,throttle_deg,'LineWidth',2);
plot(tout,xout(:,8),'r--','LineWidth',2)
title('Throttle vs Time')
legend('Data','Simulation')
%%%%PLot GPS speed
plottool(1,'Speed',12,'Time (min)','GPS Speed (m/s)');
plot(total_time_arduino_and_GPS_min,GPS_Speed,'LineWidth',2);
plot(tout,xout(:,4),'r--','LineWidth',2)
title('Velocity')
legend('Data','Simulation')
%%%%Plot Steering
plottool(1,'Steering',12,'Time (min)','Steering (deg)');
plot(total_time_arduino_and_GPS_min,steering_deg,'LineWidth',2);
plot(tout,xout(:,7)*180/pi,'r--','LineWidth',2)
title('Steering vs Time')
legend('Data','Simulation')
%%%PLot heading
plottool(1,'Heading (Deg)',12,'Time (min)','Heading (deg)');
plot(total_time_arduino_and_GPS_min,GPS_heading,'LineWidth',2);
plot(tout,wrap(xout(:,3))*180/pi+180,'r--','LineWidth',2);
title('Heading vs Time')
legend('Data','Simulation')
%%%%Here's a lat lon plot for kicks
latitude = data(:,6);
longitude = data(:,7);
plottool(1,'XvsY',12,'X (m)','Y (m)');
origin = [latitude(1);longitude(1)];
[x,y] = convertLATLON(latitude,longitude,origin);
plot(x,y,'LineWidth',2)
axis equal
plot(xout(:,1),xout(:,2),'r--','LineWidth',2)
title('Vehicle Path')
legend('Data','Simulation')
