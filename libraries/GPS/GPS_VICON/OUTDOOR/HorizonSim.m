function PTPout = HorizonSim(PTPin,NOISETYPE)

Noise = 5*pi/180;
Bias  = 5*pi/180.*(-1 + 2*rand(3,1));

%%Noise
if NOISETYPE == 1
  for i = 1:c
    PTPout(1:3,i) = Bias + PTPin(1:3,i) - Noise + (2.*Noise).*rand(3,1);
  end
else
  PTPout = PTPin;
end

