function plotcommands(arg1)
global NSTATES DELTA NOISE

if exist('arg1','var')
    states = arg1;
else
    states = 1:5;
end

%%%PLOTTING ROUTINE
data = dlmread('source/Out_Files/Meta_Command.OUT');
xout = data(:,2:end);
xout = xout';
tout = data(:,1);
dim1 = 'm';
[c,r] = size(xout);
NCOMMANDS = 5;
num_ac = round(c/NCOMMANDS);
Names = {'Z','Theta','Phi','Phi','Psi'};
ylabels = {['z(',dim1,')'],'\theta(deg)','\phi(deg)','\phi(deg)','\psi(deg)'};
units = 1;
LineWidth = 2;
colors = {'b','r','g','m','c','k'};
linetype = {'-','--','-.','-','--','-.'};
for ii = states
    if ii >= 2
      factor = 180/pi;
    else
      factor = units;
    end
    %factor = 1;
    h1 = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
    p(1) = plot(tout,factor.*xout(ii,:),'b-','LineWidth',LineWidth);
    for jj = 2:num_ac
      val = (jj-1)*NCOMMANDS+ii;
      p(jj) = plot(tout,factor.*xout(val,:),[colors{jj},linetype{1}],'LineWidth',LineWidth);
    end
    LegendNames = {'Parent','Child'};
    if exist('LegendNames','var')
      legend(p,LegendNames,'Location','NorthEastOutside')
    end
end

