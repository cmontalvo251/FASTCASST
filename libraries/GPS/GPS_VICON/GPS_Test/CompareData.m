purge

file1 = 'park_dgps_test_1_aap_1.txt';
file2 = 'park_dgps_test_1_aap_2.txt';

Data = load(file1);
lat = (2^24*Data(:,6)+2^16*Data(:,7)+2^8*Data(:,8)+Data(:,9));
lon = (2^24*Data(:,10)+2^16*Data(:,11)+2^8*Data(:,12)+Data(:,13));
time = 2^24*Data(:,2)+2^16*Data(:,3)+2^8*Data(:,4)+Data(:,5);
%time = (time-time(1))./1000;
time1 = time;
for i = 1:length(time)
    if (lat(i) >= 2^31) lat(i) = lat(i)-2^32-0; end 
    if (lon(i) >= 2^31) lon(i) = lon(i)-2^32-0; end 
end
lon1 = lon;
lat1 = lat;
l = length(lat);
aa=437.8826741382089; %//equatiorial radius^2 (Mft^2)
bb=434.9513211202460; %//polar radius^2 (Mft^2)
clat = cos(0.0000000174533*lat);
slat2 = 1.0-clat.*clat;
den = sqrt(aa*clat.*clat+bb*slat2);
north1 = 0.0174533*aa*bb./(den.*den.*den).*lat;
m = mean(north1);
east1 = 0.0174533*clat*aa./den.*lon;
e = mean(east1);

Data = load(file2);
lat = (2^24*Data(:,6)+2^16*Data(:,7)+2^8*Data(:,8)+Data(:,9));
lon = (2^24*Data(:,10)+2^16*Data(:,11)+2^8*Data(:,12)+Data(:,13));
time = 2^24*Data(:,2)+2^16*Data(:,3)+2^8*Data(:,4)+Data(:,5);
%time = (time-time(1))./1000;
time2 = time;
for i = 1:length(time)
    if (lat(i) >= 2^31) lat(i) = lat(i)-2^32-0; end 
    if (lon(i) >= 2^31) lon(i) = lon(i)-2^32-0; end 
end
lat2 = lat;
lon2 = lon;
aa=437.8826741382089; %//equatiorial radius^2 (Mft^2)
bb=434.9513211202460; %//polar radius^2 (Mft^2)
clat = cos(0.0000000174533*lat);
slat2 = 1.0-clat.*clat;
den = sqrt(aa*clat.*clat+bb*slat2);
north2 = 0.0174533*aa*bb./(den.*den.*den).*lat;
east2 = 0.0174533*clat*aa./den.*lon;

plottool(1,'NE',12,'North(ft)','East(ft)');
plot(north1-m,east1-e,'b-')
plot(north2-m,east2-e,'r-')

plottool(1,'Lat_Lon',12);
plot(lat1,lon1)
plot(lat2,lon2,'r-')


plottool(1,'Latitude',12,'Time(msec)','North(ft)');
plot(time1,[north1-m])
plot(time2,[north2-m],'r-')

plottool(1,'Latitude',12,'Time(msec)','East(ft)');
plot(time1,[east1-e])
plot(time2,[east2-e],'r-')
