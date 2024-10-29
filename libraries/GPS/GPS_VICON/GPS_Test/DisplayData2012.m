clear all
clc

Data = load('park_dgps_test_2_aap_2.txt');

PTS = [30+(37.2100/60) -(96+(20.3110/60))];

%
[row,col] = size(Data);
%
time = 2^24*Data(:,2)+2^16*Data(:,3)+2^8*Data(:,4)+Data(:,5);
time = (time-time(1))./1000;
lat = (2^24*Data(:,6)+2^16*Data(:,7)+2^8*Data(:,8)+Data(:,9));
lon = (2^24*Data(:,10)+2^16*Data(:,11)+2^8*Data(:,12)+Data(:,13));
altgps = (2^8*Data(:,14)+Data(:,15));
fix = Data(:,16);
hac = 0.1*(2^8*Data(:,17)+Data(:,18));
veld = 0.1*(2^8*Data(:,19)+Data(:,20));
speed = 0.1*(2^8*Data(:,21)+Data(:,22));
course = 0.1*(2^8*Data(:,23)+Data(:,24));
caac = 0.1*(2^8*Data(:,25)+Data(:,26));

variable1 = (2^8*Data(:,27)+Data(:,28));
variable2 = (2^8*Data(:,29)+Data(:,30));
variable3 = (2^8*Data(:,31)+Data(:,32));
variable4 = (2^8*Data(:,33)+Data(:,34));
variable5 = (2^8*Data(:,35)+Data(:,36));
variable6 = (2^8*Data(:,37)+Data(:,38));

% Next Three bytes are empty
auto = Data(:,42);
pathindex = Data(:,43);
% Terminator "!"
% Sensor Header "0x31"
roll = (2^8*Data(:,46)+Data(:,47));
pitch = (2^8*Data(:,48)+Data(:,49));
yaw = (2^8*Data(:,50)+Data(:,51));
ax = (2^8*Data(:,52)+Data(:,53));
ay = (2^8*Data(:,54)+Data(:,55));
az = (2^8*Data(:,56)+Data(:,57));
p = (2^8*Data(:,58)+Data(:,59));
q = (2^8*Data(:,60)+Data(:,61));
r = (2^8*Data(:,62)+Data(:,63));
baro = (2^8*Data(:,64)+Data(:,65));
temperature = (2^8*Data(:,66)+Data(:,67));
% Next two bytes are empty
vvel = (2^8*Data(:,70)+Data(:,71));
count = (2^8*Data(:,72)+Data(:,73));
channel1 = (2^8*Data(:,75)+Data(:,74));
channel2 = (2^8*Data(:,77)+Data(:,76));
channel3 = (2^8*Data(:,79)+Data(:,78));
channel4 = (2^8*Data(:,81)+Data(:,80));
channel5 = (2^8*Data(:,83)+Data(:,82));
channel6 = (2^8*Data(:,85)+Data(:,84));
%
for i = 1:length(time)
    if (lat(i) >= 2^31) lat(i) = lat(i)-2^32-0; end 
    if (lon(i) >= 2^31) lon(i) = lon(i)-2^32-0; end 
    if (roll(i) >= 2^15) roll(i) = roll(i)-2^16-0; end
    if (pitch(i) >= 2^15) pitch(i) = pitch(i)-2^16-0; end
    if (yaw(i) >= 2^15) yaw(i) = yaw(i)-2^16-0; end
    if (ax(i) >= 2^15) ax(i) = ax(i)-2^16-0; end
    if (ay(i) >= 2^15) ay(i) = ay(i)-2^16-0; end
    if (az(i) >= 2^15) az(i) = az(i)-2^16-0; end
    if (p(i) >= 2^15) p(i) = p(i)-2^16-0; end
    if (q(i) >= 2^15) q(i) = q(i)-2^16-0; end
    if (r(i) >= 2^15) r(i) = r(i)-2^16-0; end
    if (baro(i) >= 2^15) baro(i) = baro(i)-2^16-0; end
    if (vvel(i) >= 2^15) vvel(i) = vvel(i)-2^16-0; end
end
   
clat = cos(0.0000000174533*lat);
slat2 = 1.0-clat.*clat;
aa=437.8826741382089; %//equatiorial radius^2 (Mft^2)
bb=434.9513211202460; %//polar radius^2 (Mft^2)
den = sqrt(aa*clat.*clat+bb*slat2);
north = 0.0174533*aa*bb./(den.*den.*den).*lat;
east = 0.0174533*clat*aa./den.*lon;

lat = 0.000001*lat;
lon = 0.000001*lon;
roll = roll*180/32768;
pitch = pitch*180/32768;
yaw = yaw*180/32768;
ax = ax/4096;
ay = ay/4096;
az = az/4096;
p = (180/pi)*p*10000/32786000;
q = (180/pi)*q*10000/32786000;
r = (180/pi)*r*10000/32786000;
vvel = vvel/10.0;

corr = cos(3.1416*PTS(1,1)/180);

%
% Display
%
color(1,[1:3]) = 'k. ';
color(2,[1:3]) = 'b. ';
color(3,[1:3]) = 'y. ';
color(4,[1:3]) = 'g. ';
color(5,[1:3]) = 'c. ';
color(6,[1:3]) = 'b. ';
color(7,[1:3]) = 'r. ';
color(8,[1:3]) = 'b. ';
color(9,[1:3]) = 'k. ';
color(10,[1:3]) = 'g. ';
%
h1 = figure('Name','pos');

for i = 1:length(lon)
    plot(lon(i,1),lat(i,1),color(pathindex(i)+1,[1:3]) )
    hold on
end
plot(PTS(:,2),PTS(:,1),'rx')
daspect([1 cos(lat(1,1)/57.3) 1])
set(gca,'FontSize',16)
xlabel('Longitude')
ylabel('Latitiude')
grid on
%
h2 = figure('Name','status');
plot(time,hac,'k',time,10*fix,'r')
set(get(gca,'Children'),'linewidth',2)
set(gca,'FontSize',16)
xlabel('Time (S)')
legend('HAC (ft)','FIX*10')
axis([0 time(end) 0 40])
grid on
%
h9 = figure('Name','Altitude');
plot(time,baro,'k')
set(get(gca,'Children'),'linewidth',2)
set(gca,'FontSize',16)
xlabel('Time (s)')
ylabel('Esitmated Altitude (ft)')
grid on
%
h10 = figure('Name','Vertical Velocity');
plot(time,vvel,'k')
set(get(gca,'Children'),'linewidth',2)
set(gca,'FontSize',16)
xlabel('Time (S)')
ylabel('FT/S')
grid on
%
h12 = figure('Name','Speed');
plot(time,speed,'k')
set(get(gca,'Children'),'linewidth',2)
set(gca,'FontSize',16)
xlabel('Time (S)')
ylabel('Speed (Ft/s)')
grid on
%
h18 = figure('Name','Yaw');
plot(time,yaw,'b',time,course,'k')
set(get(gca,'Children'),'linewidth',2)
set(gca,'FontSize',16)
xlabel('Time (s)')
ylabel('(deg)')
grid on
%
h18 = figure('Name','Roll and Pitch');
plot(time,roll,'b',time,pitch,'k')
set(get(gca,'Children'),'linewidth',2)
set(gca,'FontSize',16)
xlabel('Time (s)')
ylabel('(deg)')
grid on
%
h7 = figure('Name','Rotation Rates');
plot(time,p,'r',time,q,'g',time,r,'b')
set(get(gca,'Children'),'linewidth',2)
set(gca,'FontSize',16)
xlabel('Time (S)')
ylabel('DEG/S')
legend('P','Q','R')
grid on
%
h7 = figure('Name','Channels');
plot(time,channel1,'k',time,channel2,'b',time,channel3,'r',...
     time,channel4,'k--',time,channel5,'b--',time,channel6,'r--')
set(get(gca,'Children'),'linewidth',2)
set(gca,'FontSize',16)
xlabel('Time (S)')
ylabel('u seconds')
legend('CH1','CH2','CH3','CH4','CH5','CH6')
grid on
%