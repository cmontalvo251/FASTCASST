purge

file = 'Meta_Constant.MC';
out  = 'Meta_ConstantV10.MC';

%IMPORT
fin = importdata(file);
header = fin.textdata;
data = fin.data;
data = data(:,1:30); %%Grab states
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

%WRITE
%header{1}(1) = num2str(ii);
dlmwrite(out,header,'delimiter','');
dlmwrite(out,data,'-append','delimiter',' ')