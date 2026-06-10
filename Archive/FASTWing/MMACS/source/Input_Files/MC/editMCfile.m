purge

file = 'Meta_Nominal_V10_Wslice_slice1.MC';
out  = 'Meta_Nominal_V10_Wslice_slice11.MC';

%IMPORT
fin = importdata(file);
header = fin.textdata;
data = fin.data;
data = data(1:200,1:30); %%Grab states
[r,c] = size(data);
temp = [data];
%data = [];

CHANGE = 0;
ADD = 0;

%%%Change something
if CHANGE
  %param = [0.2,0.75,1.25,2];
  %param = [1.2,1.4,1.6];
  %param = [0.2,0.4,0.6];
  %param = [0,0.005,0.01];
  %param = [100,150,200];
  param = [0.0125]*8;%,3,6];
  %param2 = [0,0.0035,0.007];
  loc = 33;
  %loc = [68];
  %loc = [29,30,31];
  %loc2 = [32,33,34];
  for ii = 1:length(param)
    temp(:,loc) = param(ii);
    %temp(:,loc2) = param2(ii);
    data = [data;temp];
  end
end
%%%add something
if ADD
  %delta = [-3,5,3,3];
  %delta = [200,-200,-350];
  delta = -5;
  %delta = [+3];
  loc = [7,22];
  %loc = [3,17];
  for ii = 1:length(delta)
    temp(:,loc) = temp(:,loc)+delta(ii);
    data = [data;temp];
  end
end

[r,c] = size(data);

data = [data,zeros(r,15)]; %%add all of the wind levels
%31 = ICONSTANT
%32,33,34=ICONTSTANTSCALE
%35,36,37=FREQ
%38=ITURB
%39,40,41=TURBLEVEL
%42=IWRF
%43,44,45=IWRFSCALE

%%%Constant Winds
data(:,31) = 1;
%%Turbulence
data(:,38) = 1;
%%WRF on
data(:,42) = 1;

%w = 0.25:0.25:3;
w = 0.8;
f = linspace(0,0.1,40);
f = f(40);
%V = 15:1:30;
V = 20;
extra = data;
data = [];
for ii = 1:length(w)
    for jj = 1:length(f)
      for kk = 1:length(V)
	%%%Parent Velocity
	%extra(:,7) = V(kk);
	%%Child Velocity
	%extra(:,22) = V(kk);
	%%Iconstant
	%extra(:,32) = w(ii)*0;
	%extra(:,33) = w(ii)*0;
	extra(:,34) = w(ii);
	%%Frequency
	extra(:,35:37) = f(jj);
	%%Turbulence
	%extra(:,39:41) = w(ii);
	%%wrf
	%extra(:,43:45) = w(ii);
	data = [data;extra];
      end
    end
end

%WRITE
header{1} = [num2str(r*length(w)*(length(f))),' !Number_of_Cases'];
dlmwrite(out,header,'delimiter','');
dlmwrite(out,data,'-append','delimiter',' ')