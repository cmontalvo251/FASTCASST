function plotcontrols(filename)
%%%PLOTTING ROUTINE
LegendNames = {'1st Aircraft','2nd Aircraft','3rd Aircraft'};
%LegendNames = {'1st Aircraft','2nd Aircraft'};

data = dlmread(filename);
fcsout = data(:,2:end);
fcstout = data(:,1);
[r,c] = size(fcsout);
num_ac = round(c/4);
dim1 = 'm';
Names = {'Elevator Deflection (deg)','Aileron Deflection (deg)','DR(deg)','DT'};
ylabels = Names;
units = 1;
LineWidth = 2;
colors = {'b','r','g','m','c','k'};
colors = {'k','k','k','k'};
linetype = {'-','--','-.','s-','--','-.'};
for ii = 1:4
  if ii ~= 4
    factor = 180/pi;
  else
    factor = 1;
  end
  h1 = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
  p(1) = plot(fcstout,factor.*fcsout(:,ii),colors{1},'LineWidth',LineWidth);
  for jj = 2:num_ac
    val = (jj-1)*4 + ii;
    p(jj) = plot(fcstout,factor.*fcsout(:,val),[colors{jj},linetype{jj}],'LineWidth',LineWidth);
  end
  %xlim([0 50])
  if exist('LegendNames','var')
    legend(p,LegendNames)
  end
end
