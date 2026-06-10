data = dlmread('source/Out_Files/Meta_Blend.OUT');
blendout = data(:,2:end);
blendtout = data(:,1);
dim1 = 'm';
Names = {'THOLD','TY','PSIPATH','PSIY','WAYPOINTOFFSET','WAYPOINT'};
ylabels = Names;
units = 1;
LineWidth = 2;
colors = {'b','r','g','m','c','k'};
linetype = {'-','--','-.','-','--','-.'};
for ii = 1:6
  h1 = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
  plot(blendtout,blendout(:,ii),'b-','LineWidth',LineWidth);
end
