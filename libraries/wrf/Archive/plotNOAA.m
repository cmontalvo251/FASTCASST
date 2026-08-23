purge

%%Read Raw Data
fid = fopen('Wind_Data/NOAA.txt');
all = {};
cities = {};
allstates = {};
data = [];
r = 276;
for ii = 1:r
  row = fgetl(fid);
  if ii ~= 1
    city = row(1:33);
    mph = str2num(row(33:end));
    data = [data;mph];
    cities = [cities;city];
    %%Figure out state
    loc = find(city==',')+1;
    state = city(loc:end);
    state(state==' ')=[];
    state(state=='.')=[];
    allstates = [allstates;state];
  end
end
%%Delete years column
data = data(:,2:end);
%%Delete average column
data = data(:,1:end-1);
%%Convert to M/S
dataMS = data.*(1/3600)*(5280)*(1/3.28);

plottool(1,'Winddata All',12,'Month','Speed(m/s)');
plot([1:12],dataMS)
xlim([1 12])

%%Compile by state
dataState = [];
states = {};
statedata = zeros(1,12);
numcities = 0;
statename = 'AL';
for ii = 1:r-1
  if statename == allstates{ii}
    numcities = numcities + 1;
    statedata = statedata + dataMS(ii,:);
  else
    %%Compile previous state
    statedata = statedata./numcities;
    dataState = [dataState;statedata];
    states = [states;statename];
    %%Reset everything
    statename = allstates{ii};
    statedata = dataMS(ii,:);
    numcities = 1;
  end
end

plottool(1,'States',12,'Month','Speed(m/s)');
plot([1:12],dataState)
xlim([1 12])
loc = 1;
for ii = 1:length(states)
  text(loc,dataState(ii,loc),states{ii})
  loc = loc + 1;
  if loc == 13
    loc = 1;
  end
end

%%Histogram
plottool(1,'Histogram',12,'WindSpeed(m/s)','Count');
vec = unwrapmatrix(dataState);
[h] = hist(vec,[1:10]);
hist(vec,[1:10]);
Pwinds = h./sum(h);
Pconn = [99 95 84 70 50 32 10 0 0 0]./100;
EffectConnection = sum(Pwinds.*Pconn).*100

%%Average for US
dataAVG = mean(dataState);
plottool(1,'Average',12,'Month','Speed(m/s)');
plot([1:12],dataAVG)
xlim([1 12])

%%Average Per State over 1 yr
dataAVGYear = mean(dataState');
plottool(1,'AverageYear',12,'State','Speed(m/s)');
plot([1:52],dataAVGYear)
set(gca,'XTick',[1:52])
set(gca,'XTickLabel',states)

%%Average Overall
AVGWS = mean(dataAVGYear)


