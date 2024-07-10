function GPSout = GPSsim(GPSin,NOISETYPE)
%%%This function will add in GPS errors
%%The input mus be a vector with 3 rows and N columns
%The rows are x,y,z,xdot,ydot,zdot respectively

[r,c] = size(GPSin);
%%%NOISE PARAMETERS(ALL STANDARD DEVIATIONS)
xposNoise = 0.85;
yposNoise = 0.85;
zposNoise = 1.9;
positionNoise = [xposNoise;yposNoise;zposNoise];
xposbias = 1.06; %meters
yposbias = 1.06;
zposbias = 2.38;
positionBias = [xposbias;yposbias;zposbias];

%%%%%%%XYZ NOISE%%%%%%%%

XYZin = GPSin(1:3,:);
epsilon = 0.9999;
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
    xnoise = positionNoise.*N01;
    xbias = xbias.*epsilon + positionBias.*sqrt(1-epsilon^2).*N01;
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

GPSout = XYZout;
