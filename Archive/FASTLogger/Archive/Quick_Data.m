clear
clc
close all

filename = 'FAST_03.TXT';
delimiter = ' ';
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);
Hour = dataArray{:, 1};
Minute = dataArray{:, 2};
Second = dataArray{:, 3};
LastPrint = dataArray{:, 4};
Fix = dataArray{:, 5};
Latitude = dataArray{:, 6};
Longitude = dataArray{:, 7};
Speed = dataArray{:, 8};
Angle = dataArray{:, 9};
Altitude = dataArray{:, 10};
X_Accel = dataArray{:, 11};
Y_Accel = dataArray{:, 12};
Z_Accel = dataArray{:, 13};
X_Mag = dataArray{:, 14};
Y_Mag = dataArray{:, 15};
Z_Mag = dataArray{:, 16};
X_Gyro = dataArray{:, 17};
Y_Gyro = dataArray{:, 18};
Z_Gyro = dataArray{:, 19};
Temperature = dataArray{:, 20};
Pressure = dataArray{:, 21};
Ground_Pressure = mean(Pressure(1:100,:));


%%%%Since we did this experiment indoors we won't have any GPS data. Thus I
%%%%will be using LastPrint as the time vector although remind me to give
%%%%you a MATLAB code that will fuse ArduinoTime with GPS time. 
time_vec = LastPrint-LastPrint(1); 

%%%Plot Magnetometer Data
figure()
MAG = [X_Mag,Y_Mag,Z_Mag];
plot(time_vec,MAG)
xlabel('Time (sec)')
ylabel('Mag Field (nF)')
legend('X','Y','Z')

%%%Plot Accelerometer Data
ACCEL = [X_Accel,Y_Accel,Z_Accel];
figure()
plot(time_vec,ACCEL)
xlabel('Time (sec)')
ylabel('Acceleration (m/s^2)')
legend('X','Y','Z')

%%%Plot Rate Gyro Data
RATE = [X_Gyro,Y_Gyro,Z_Gyro];
figure()
plot(time_vec,RATE)
xlabel('Time (sec)')
ylabel('Rate Gyro (rad/s)')
legend('X','Y','Z')

%%%%Compute Phi,Theta and Psi using Multiple Methods.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()

%%%Phi and Theta can be computed using the accelerometer using the
%%%relationship of tranforming z_inertial into the body frame. 
phi_accel = atan2(Y_Accel,Z_Accel);
theta_accel = atan2(-X_Accel,Y_Accel.*sin(phi_accel) + Z_Accel.*cos(phi_accel));

%%%%After running this code a few times I realized that when the pitch
%%%%angle approaches 90 degrees we run into a singularity problem. As such
%%%%I multiplied phi by cos(theta_accel)^2
phi_accel = phi_accel.*cos(theta_accel).^2;

%%%It is not possible to compute heading with the accelerometer

p1 = plot(time_vec,phi_accel*180/pi,'b-','LineWidth',2);
hold on
plot(time_vec,theta_accel*180/pi,'r-','LineWidth',2)

%%%%Finally the magnetometer can be used to obtain psi using
%%%%the dot product relationship between the initial value of the mag field
%%%%and the mag field obtained during flight. Turns out Adafruit only gets
%%%%psi rather than getting phi or theta. I guess that makes sense since a
%%%%dot product only returns one angle

%%%One thing I noticed with the magnetometer is that the sensor took about
%%%0.2 seconds to calibrate. So I'm going to calibrate the magnetometer by
%%%averaging the first 5 seconds of data and then setting the first 5
%%%seconds to that value
loc = find(time_vec > 5,1);
Z_Mag_AVG = mean(Z_Mag(1:loc));
Y_Mag_AVG = mean(Y_Mag(1:loc));
X_Mag_AVG = mean(X_Mag(1:loc));
X_Mag(1:loc) = X_Mag_AVG;
Y_Mag(1:loc) = Y_Mag_AVG;
Z_Mag(1:loc) = Z_Mag_AVG;

psi_mag = atan2(Z_Mag .* sin(phi_accel) - Y_Mag .* cos(phi_accel), X_Mag .* cos(theta_accel) + Y_Mag .* sin(theta_accel) .* sin(phi_accel) + Z_Mag .* sin(theta_accel) .* cos(phi_accel));

%%%Finally I'm pretty sure this equation assumes a certain magnetic field reference
%%%point so I'm going to calibrate the reading to zero initially
psi_mag = psi_mag - psi_mag(1);

%%%The singularity in pitch causes the yaw angle to go kind of wonky as
%%%well so I've multiplied yaw_mag by cos(theta)^2
psi_mag = psi_mag.*cos(theta_accel).^2;

plot(time_vec,psi_mag*180/pi,'g-','LineWidth',2)

%%%Phi, theta and Psi can be computed using the rate gyro by integrating
%%%the signals. This will be very noise and will most likely require a
%%%filter. I suggest a complimentary filter but we shall see how that goes.
%%%We're gonna need initial conditions. Let's also use trapezoidal rule for
%%%integration to reduce noise
phi_gyro = zeros(length(X_Gyro)-1,1);
theta_gyro = phi_gyro;
psi_gyro = theta_gyro;

phi_gyro(1) = phi_accel(1);
theta_gyro(1) = theta_accel(1);
psi_gyro(1) = psi_mag(1);

for idx = 1:length(X_Gyro)-1
    phi_gyro(idx+1) = phi_gyro(idx) + 0.5*(X_Gyro(idx) + X_Gyro(idx+1))*(time_vec(idx+1)-time_vec(idx));
    theta_gyro(idx+1) = theta_gyro(idx) + 0.5*(Y_Gyro(idx) + Y_Gyro(idx+1))*(time_vec(idx+1)-time_vec(idx));
    psi_gyro(idx+1) = psi_gyro(idx) + 0.5*(Z_Gyro(idx) + Z_Gyro(idx+1))*(time_vec(idx+1)-time_vec(idx));
end

plot(time_vec,phi_gyro*180/pi,'b--','LineWidth',2)
plot(time_vec,theta_gyro*180/pi,'r--','LineWidth',2)
plot(time_vec,psi_gyro*180/pi,'g--','LineWidth',2)

%%%%%It should then be possible to stitch the data together using a
%%%%%weighted average technique. Unfortunately though you can see when you
%%%%%plot this that the Z_Gyro signal for some reason had a ton of bias and
%%%%%unfortunately we integrated that bias so it just got worse. This is
%%%%%why magnetometers are used to get rid of that bias. It looks like roll
%%%%%and pitch are in good agreement but even they have some integration
%%%%%bias. I would say for now just use the phi and theta computed by accel
%%%%%and yaw from the mag. With more time I can figure out how to create a
%%%%%filtering algorithm that attempts to measure the bias in the
%%%%%magnetometer and correct for it.


xlabel('Time (sec)')
ylabel('Angles (Deg)')
legend('Roll (Accel)','Pitch (Accel)','Yaw (Mag)','Roll (Gyro)','Pitch (Gyro)','Yaw (Gyro)')

