%%%Compute wingtip velocity
data = dlmread('source/Out_Files/Meta.OUT');

xout = data(:,2:end);

ucg = xout(:,7);
vcg = xout(:,8);
wcg = xout(:,9);
pcg = xout(:,10);
qcg = xout(:,11);
rcg = xout(:,12);

uwing = ucg;
vwing = vcg;
wwing = wcg;

rwing = [0;1.02;0];

%Vwing = Vcg + omega x rwing
for ii = 1:length(ucg)
   uwing(ii) = uwing(ii) + [0 -rcg(ii) qcg(ii)]*rwing;
   vwing(ii) = vwing(ii) + [rcg(ii) 0 -pcg(ii)]*rwing;
   wwing(ii) = wwing(ii) + [-qcg(ii) pcg(ii) 0]*rwing;
end
%%%Now get delta from other wing
vdelta = 0.*vwing; %%assume side velocity help regardless of the direction
udelta = uwing - 19.93;
wdelta = wwing - 1.654;

figure()
plot(udelta)
figure()
plot(vdelta)
figure()
plot(wdelta)

%%%Total Velocity
Vwing = sqrt(udelta.^2 + vdelta.^2 + wdelta.^2);
figure()
plot(Vwing)

MaxWing = max(Vwing)

