function dzdt = Derivatives(t,z)
global AUTOPILOT WPcounter tGPS lat_GPS lon_GPS psi_GPS GPS_speed servoAcommand deltaTcommand
global total_time_arduino_and_GPS_min throttle_deg steering_deg xmeasured ymeasured psimeasured umeasured
%%%Extract States
x = z(1);
y = z(2);
psi = z(3);
u = z(4);
v = z(5);
r = z(6);
servoA = z(7);
deltaT = z(8);
xDELAY = z(9);
yDELAY = z(10);
psiDELAY = z(11);
uDELAY = z(12);

%%%Parameters
g = 9.81; %%%m/s^2 - Gravitational constant
m = 1.852; %kg - measured using a scale
L = 0.7; %%m Length between front and rear wheel
%Actual distance is 0.3 m, but cause inaccurate simulation
cb = 0.003; %%%Bearing coefficient from Lisa

%%%Time Constants
tau = 4/0.1;        %Steering Time Constant - Still need to measure
tauT = 4/0.5;      %Throttle Time Constant - Still need to measure
c = 2.1;        %Parameter fit using open loop data
alfa = 0.469;   %Parameter fit using open loop data

%%%%Origin to convert GPS to X,Y
origin = [30.697877,-88.19129]; %%%Random GPS coordindate from the intramural fields
GPS_HEADING_OFFSET = 0; %%%GPS is definitely not offset by anything. 
%%If you walk north your GPS heading is 0 or 360. East or along the y
%%coordinate gives you a GPS heading of 90 degrees

%%%%Simulate the GPS Sensor
%%%I think GPS has a pretty bad delay so we can do 1 of two things
%%%1.) We can pull the actual x and y here and then save it for later
%%%2.) Code in a first order filter. Yea I like the first order filter
%%%idea
%%%Ok simulate all the GPS variables using first order filters
xy_T_settling = 3; %%%settling time for x and y
psi_T_settling = 5; %%%settling time for psi
a_xyu = 4/xy_T_settling; %%%x,y take about 3 seconds but it goes faster if you set GPS update rate to 10Hz
a_psi = 4/psi_T_settling; %%%Turns out psi is even slower because it uses a filter.
%%%If psi turns out to be super freaking slow it means we may need to add
%%%in a magnetometer and blend GPS+magnetometer
xDELAYdot = a_xyu*(x-xDELAY);
yDELAYdot = a_xyu*(y-yDELAY);
psiDELAYdot = a_psi*(psi-psiDELAY);
uDELAYdot = a_xyu*(u-uDELAY);

%%%If we're ready to sample GPS go ahead and pull it
if t >= tGPS
  GPS_UPDATE_RATE = 1/1; %%1/10 is 10Hz and 1/1 is Hz. 
  %%%It's possible to change the update rate on the arduino so keep that in mind if you want to change this to 10 Hz.
  tGPS = tGPS + GPS_UPDATE_RATE;
  %%%Perfect measurement
  %xmeasured = x;
  %ymeasured = y;
  %psimeasured = psi;
  %umeasured = u;
  %%%%In reality We will need to take GPS and convert it to x and y so we
  %%%%will mimick this by taking x and y and polluting it and then converting
  %%%%back to x and y
  [lat,lon] = convertXY2LATLON(xDELAY,yDELAY,origin);    
  %%%Now we need to pollute the data
  NM2FT=6076.115485560000;
  FT2M=0.3048;
  alf = 60*NM2FT*FT2M;
  %%%To pollute the data we need to understand that lat,lon is in degrees and
  %%%x,y is in meters. GPS is accurate to about 2.38 m or in this case
  %%% 2.38/alf where alf is a conversion factor from meters to deg
  %%%Just to make sure our algorithm is doing well let's make our error
  %%%really bad like 5 meters
  GPS_ACCURACY = 2.38;
  noise = GPS_ACCURACY/alf;
  lat_GPS = lat+(1-2*rand)*noise;
  lon_GPS = lon+(1-2*rand)*noise;
  %%%We also need to understand that GPS is going to give us a heading angle
  %%%between 0 and 360 degrees and it's offset by 180 I THINK. That is when we are
  %%%driving with a heading of 0 degrees the gps is going to return 180
  %%%degrees. We need to verify this in simulation though of course.
  %%%Let's also assume we have about 10 degrees of error
  GPS_HEADING_ERROR = 10;
  psi_GPS = (psiDELAY*180/pi + GPS_HEADING_OFFSET)+GPS_HEADING_ERROR*(1-2*rand); 
  %%%%GPS is also going to give us the speed in knots
  %%%Again with some 10% error
  %%%We probably are going to have a bias too of say 1 knot
  GPS_SPEED_ERROR = 0.10;
  GPS_SPEED_BIAS = 1;
  GPS_speed = (uDELAY/0.51444)*(1+GPS_SPEED_ERROR*(1-2*rand))+GPS_SPEED_BIAS*(1-2*rand);
end

%%Control
%%%Steering Controller
if AUTOPILOT
    %Assuming GPS gives us lat and lon we need to convert it to x and y
    [xmeasured,ymeasured] = convertLATLON(lat_GPS,lon_GPS,origin);
    %%%GPS will also give us heading
    psimeasured = (psi_GPS - GPS_HEADING_OFFSET)*pi/180;
 
    %%%Waypoint vectors
    WPx = [200 200 0 0];
    WPy = [0 200 200 0];
    xcommand = WPx(WPcounter);
    ycommand = WPy(WPcounter);
    GPS_MINIMUM = 20; %%%Once vehicle gets within this value of the waypoint
    %%%then move on to the next waypoint
    if sqrt((xcommand-xmeasured)^2 + (ycommand-ymeasured)^2) < GPS_MINIMUM
        WPcounter = WPcounter + 1;
        if WPcounter > length(WPx)
	    WPcounter = 1;
            %WPcounter = length(WPx);
        end
    end
    psic = atan2(ycommand-ymeasured,xcommand-xmeasured);
    dpsi = delpsi(psimeasured,psic);
    kp = -10;
    kd = 0;
    servoAcommand = kp*dpsi + kd*r + 93;
    %servoAcommand = 93;
    %%%Saturation Block
    if servoAcommand > 120
        servoAcommand = 120;
    elseif servoAcommand < 61
        servoAcommand = 61;
    end
else    
    tinterp = t/60;
    if tinterp <= total_time_arduino_and_GPS_min(1)
        servoAcommand = steering_deg(1);
    elseif tinterp >= total_time_arduino_and_GPS_min(end)
        servoAcommand = steering_deg(end);
    else
        servoAcommand = interp1(total_time_arduino_and_GPS_min,steering_deg,tinterp);
    end
end

%servoAcommand = servoAcommand*pi/180;
servoAdot = tau*(servoAcommand*pi/180 - servoA);

deltaNW = (servoA-93*pi/180)*(22.5/30);
deltaNWdot = servoAdot*(22.5/30);
%%%%Throttle

if AUTOPILOT
    %%%GPS is going to give us speed in knots
    umeasured = GPS_speed*0.514444;
    ucommand = 10; %%%Let's drive at 10m/s
    deltaTcommand = 5*(ucommand-umeasured) + 88;
    %deltaTcommand = 126;
    %%%Saturation Blocks
    if deltaTcommand > 126
        deltaTcommand = 126;
    elseif deltaTcommand < 88
        deltaTcommand = 88;
    end
    %deltaTcommand = 126;
else
    if tinterp <= total_time_arduino_and_GPS_min(1)
        deltaTcommand = throttle_deg(1);
    elseif tinterp >= total_time_arduino_and_GPS_min(end)
        deltaTcommand = throttle_deg(end);
    else
        deltaTcommand = interp1(total_time_arduino_and_GPS_min,throttle_deg,tinterp);
    end
end

FT = alfa*(deltaT-88);
if FT < 0 %FT cannot be negative we aren't coding in brakes
    FT = 0;
end
deltaTdot = tauT*(deltaTcommand-deltaT);
utilde = cos(deltaNW)*u + sin(deltaNW)*r*L;
Ff = -c*u;
XRW = -cb*m*g*sign(u);
XNWtilde = -cb*m*g*sign(utilde);
XNW = cos(deltaNW)*XNWtilde;

X = XRW + Ff + FT + XNW;

udot = X/m;
xdot = cos(psi)*u - sin(psi)*v;
ydot = sin(psi)*u + cos(psi)*v;
psidot = r;
vdot = 0;
rdot = (deltaNWdot*sec(deltaNW)^2*u+tan(deltaNW)*udot)/L;

dzdt = [xdot;ydot;psidot;udot;vdot;rdot;servoAdot;deltaTdot;xDELAYdot;yDELAYdot;psiDELAYdot;uDELAYdot];
