function GPSout = GPSsim(GPSin,NOISETYPE)
global GPSupdate
%%%This function will add in GPS errors
%%The input mus be a vector with 6 rows and N columns
%The rows are x,y,z,xdot,ydot,zdot respectively

[r,c] = size(GPSin);
if r ~= 3
  disp('GPSin does not have 6 rows')
  return
end
%%%NOISE PARAMETERS(ALL STANDARD DEVIATIONS)
xposNoise = 0.85*3.28; %%ft
yposNoise = 0.85*3.28; %%ft
zposNoise = 1.9*3.28; %%ft
positionNoise = [xposNoise;yposNoise;zposNoise];
xposbias = 1.06*3.28; %%ft
yposbias = 1.06*3.28; %%ft
zposbias = 2.38*3.28; %%ft
positionBias = [xposbias;yposbias;zposbias];
xdotdev = 0.68*3.28; %%ft/s
ydotdev = 0.68*3.28; %%ft/s
zdotdev = 1.52*3.28; %%ft/s
velNoise = [xdotdev;ydotdev;zdotdev];

%%%%%%%XYZ NOISE%%%%%%%%

XYZin = GPSin(1:3,:);
tau = 10000000; %this is an inherent GPS parameter and can be changed
epsilon = exp(-GPSupdate/tau);
XYZout = XYZin;
if NOISETYPE == 2 %%Random walk
  %%Set the initial noise in the system
  N01 = randn(3,1);
  xbias = positionBias.*N01;
  xnoise = randn(3,1).*positionNoise;
  for i = 1:c
    %%Position Noise
    XYZout(1:3,i) = XYZin(1:3,i) + xbias + xnoise;
    N01 = randn(3,1);
    xbias = N01.*positionBias;
    xnoise = xnoise.*epsilon + positionNoise.*sqrt(1-epsilon^2).*N01;
  end
end
if NOISETYPE == 1 %%White Noise
  %%Position Noise
  for i = 1:c
    XYZout(1:3,i) = XYZin(1:3,i) - positionNoise + (2.*positionNoise).*rand(3,1);
  end
end
if NOISETYPE == 0 %%off
  XYZout = XYZin;
end

%%%%%%%%%%%%%%%%%%%

%%%%XYZdot NOISE%%%%%%

XYZdotin = GPSin(4:6,:);
tau = 10;
epsilon = exp(-GPSupdate/tau);
XYZdotout = XYZdotin;
if NOISETYPE == 2 %%Random walk
  %%Set the initial noise in the system
  xdotnoise = -velNoise + (2*velNoise).*rand(3,1);
  xdotbias = [0;0;0];
  for i = 1:c
    %%Position Derivative Noise
    XYZdotout(1:3,i) = XYZdotin(1:3,i) + xdotnoise + xdotbias;
    N01 = randn(3,1);
    xdotnoise = xdotnoise.*epsilon + velNoise.*sqrt(1-epsilon^2).*N01;
    xdotbias = [0;0;0];
  end
end
if NOISETYPE == 1 %%White Noise
  %%Position Derivative Noise
  for i = 1:c
    XYZdotout(1:3,i) = XYZdotin(1:3,i) - velNoise + (2*velNoise).*rand(3,1);
  end
end
if NOISETYPE == 0 %%noise off
  XYZdotout = XYZdotin;
end

%%%%%%%%%%%%%%%%%%%%%%

GPSout = [XYZout;XYZdotout];

