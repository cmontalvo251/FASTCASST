file = 'Meta_Constant_QueuedV10.MC';
out  = 'Meta_Nominal_V10.MC';


%IMPORT
fin = importdata(file);
header = fin.textdata;
data = fin.data;
data = data(1:200,1:45); %%Grab states
[r,c] = size(data);

for ii = 1:r
  %%Vary psi of parent from -pi to pi
  psip = -pi + 2*pi*rand();
  data(ii,6) = psip;
  %%Make child psi equal to parent psi
  psic = psip;
  data(ii,21) = psic;
  %%Rotate x and y by psic
  xc0 = data(ii,16);
  yc0 = data(ii,17);
  xc = xc0*cos(psic) - yc0*sin(psic);
  yc = xc0*sin(psic) + yc0*cos(psic);
  data(ii,16) = xc;
  data(ii,17) = yc;
end

%WRITE
dlmwrite(out,header,'delimiter','');
dlmwrite(out,data,'-append','delimiter',' ')