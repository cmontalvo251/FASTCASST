%%%PLOTTING ROUTINE
data = dlmread('source/Out_Files/Meta_Control.OUT');
fcsout = data(:,2:end);
fcstout = data(:,1);
[r,c] = size(fcsout);
num_ac = round(c/4);
dim1 = 'm';
Names = {'DE(deg)','DA(deg)','DR(deg)','DT'};
ylabels = Names;
units = 1;
LineWidth = 2;
colors = {'b','r','g','m','c','k'};
linetype = {'-','--','-.','-','--','-.'};
for ii = 1:4
  if ii ~= 4
    factor = 180/pi;
  else
    factor = 1;
  end
  h1 = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
  p(1) = plot(fcstout,factor.*fcsout(:,ii),'b-','LineWidth',LineWidth);
  for jj = 2:num_ac
    val = (jj-1)*4 + ii;
    p(jj) = plot(fcstout,factor.*fcsout(:,val),[colors{jj},linetype{1}],'LineWidth',LineWidth);
  end
  if exist('LegendNames','var')
    legend(p,LegendNames,'Location','NorthEastOutside')
  end
end
