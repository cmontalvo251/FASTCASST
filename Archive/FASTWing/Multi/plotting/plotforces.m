%%%PLOTTING ROUTINE

  data = dlmread('source/Out_Files/Meta_Force.OUT');
  forceout = data(:,2:end);
  forcetout = data(:,1);
  [r,c] = size(forceout);
  num_ac = round(c/8)
  dim1 = 'm';
  Names = {'AeroF','AeroM','Lift','Drag'};
  ylabels = Names;
  units = 1;
  LineWidth = 2;
  colors = {'b','r','g','m','c','k'};
  linetype = {'-','--','-.','-','--','-.'};
  for ii = 1:2
    h1 = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
    s = (ii-1)*3 + 1;
    e = (ii-1)*3 + 3;
    plot(forcetout,forceout(:,s:e),'b-','LineWidth',LineWidth);
    for jj = 2:num_ac
      s = 8*(jj-1) + (ii-1)*3 + 1;
      e = 8*(jj-1) + (ii-1)*3 + 3;
      plot(forcetout,forceout(:,s:e),[colors{jj},linetype{1}],'LineWidth',LineWidth);
    end
  end
