purge
global nominal AEROMODEL

x0 = [0 0 -200 0 0 0 15.26 15.26 0 0 0 -3]';
nominal = x0;
e = zeros(4,1);
AEROMODEL = 1;
[Astatedot0,controls] = ACdot(0,x0,e);
AEROMODEL = 0;
[statedot0,controls] = ACdot(0,x0,e);

Apanel = zeros(12,12);
Apoint = zeros(12,12);
del = 1e-4;
for jj = 1:length(x0)
  statep = x0;
  statep(jj) = statep(jj) + del;
  AEROMODEL = 1;
  [statedot,controls] = ACdot(0,statep,e);
  Apanel(:,jj) = (statedot-Astatedot0)./del
  AEROMODEL = 0;
  [statedot,controls] = ACdot(0,statep,e)
  Apoint(:,jj) = (statedot-statedot0)./del;
end

Apanel
Apoint
















