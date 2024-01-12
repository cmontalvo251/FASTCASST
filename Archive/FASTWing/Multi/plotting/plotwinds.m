data = dlmread('source/Out_Files/Meta_Wind.OUT');
windout = data(:,2:end);
windout(1,:) = windout(2,:);
windtout = data(:,1);
[r,c] = size(windout);
num_ac = round(c/3);
dim1 = 'm';
Names = {'U','V','W'};
ylabels = Names;
units = 1;
LineWidth = 2;
colors = {'b','r','g','m','c','k'};
linetype = {'-','--','-.','-','--','-.'};
for ii = 1:3
  h1 = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
  plot(windtout,windout(:,ii),'b-','LineWidth',LineWidth);
  for jj = 2:num_ac
    val = (jj-1)*3+ii;
    plot(windtout,windout(:,val),[colors{jj},linetype{1}],'LineWidth',LineWidth);
  end
  if exist('LegendNames','var')
    legend(p,LegendNames,'Location','NorthEastOutside')
  end
end
