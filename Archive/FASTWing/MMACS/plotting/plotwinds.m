data = dlmread('source/Out_Files/Meta_Wind.OUT');
windout = data(:,2:end);
windout(1,:) = windout(2,:);
windtout = data(:,1);
[r,c] = size(windout);
NPANELS = 6;
num_ac = round(c/(6*NPANELS));
dim1 = 'm';
Names = {'U','V','W'};
ylabels = Names;
units = 1;
LineWidth = 2;
colors = {'b','r','g','m','c','k','y'};
linetype = {'-','--','-.','-','--','-.'};
for ii = 1:3
  h1 = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
  for jj = 1:num_ac
      for kk = 1:2%kk = 1:(NPANELS*2)
        val = ii + (kk-1)*3 + (jj-1)*NPANELS*6;
        plot(windtout,windout(:,val),[colors{kk},linetype{jj}],'LineWidth',LineWidth);
      end
  end
  if exist('LegendNames','var')
    legend(p,LegendNames,'Location','NorthEastOutside')
  end
end
