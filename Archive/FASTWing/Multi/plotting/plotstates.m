function plotstates(arg1,filename)
global NSTATES DELTA NOISE DUAL mass I_b I_b_inv num_ac rcg2ac

close all

if exist('arg1','var')
    states = arg1;
else
    states = 1:15;
end
if ~exist('filename','var')
    filename = 'Out_Files/Meta3_3';
end

%%%PLOTTING ROUTINE
data = dlmread(filename);
noise = dlmread('Out_Files/Meta_Noise.OUT');
%noise = dlmread('/home/carlos/Work/Grad_Research/Meta-Aircraft/Archive_MMACS/MMACS/MMACSNOMINALAPRIL/Out_Files/Meta.OUT');
xout = data(:,2:end);
xnoise = noise(2:end,2:end);
xout = xout';
xnoise = xnoise';
tout = data(:,1);
tnoise = noise(2:end,1);
dim1 = 'm';
[c,r] = size(xout);
num_ac = round(c/NSTATES);

%%%Skip rate
skip = 1;
xout = xout(:,1:skip:end);
tout = tout(1:skip:end);

Names = {'X','Y','Z','Phi','Theta','Psi','U','V','W','P','Q','R','zint','xint','yint'};
ylabels = {['x(',dim1,')'],['y(',dim1,')'],['z(',dim1,')'],'\phi(deg)','\theta(deg)','\psi(deg)',['u(',dim1,'/s)'],['v(',dim1,'/s)'],['w(',dim1,'/s)'],'p(rad/s)','q(rad/s)','r(rad/s)','zint','xint','yint'};
units = 1;
LineWidth = 2;
%colors = {'b','r','g','m','c','k','y'};
colors = {'k','k'};
linetype = {'-','--','-.','-','--','-.','-','--','-.','-'};
for ii = states
  if ii <= 15
    if ii >= 4 && ii <= 6
      factor = 180/pi;
    else
      factor = units;
    end
    if ii >= 10
      factor = 1;
    end
    %factor = 1;
    h1 = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
    p(1) = plot(tout,factor.*xout(ii,:),[colors{1},linetype{1}],'LineWidth',LineWidth);
    if ii == 1
        %plot(tout,20.*tout*cos(xout(6,1)),'r--','LineWidth',LineWidth);
    end
    if ii == 2
        %plot(tout,20.*tout*sin(xout(6,1)),'r--','LineWidth',LineWidth); 
    end
    if NOISE
      plot(tnoise,factor.*xnoise(ii,:),'b--','LineWidth',LineWidth);
    end
    %xlim([0 50])
    for jj = 2:num_ac
      val = (jj-1)*NSTATES+ii;
      jdx = jj;
      while jdx > 7
          jdx = jdx - 7;
      end
      p(jj) = plot(tout,factor.*xout(val,:),[colors{1},linetype{jj}],'LineWidth',LineWidth);
      if NOISE
    	plot(tnoise,factor.*xnoise(val,:),[colors{jdx},linetype{2}],'LineWidth',LineWidth);
      end
    end
%     if ii <= 12
%         plot(tout,factor.*xoutNDOF(:,ii),'k-','LineWidth',3);
%     end
    LegendNames = {'1st Aircraft','2nd Aircraft','3rd Aircraft'};
    if exist('LegendNames','var')
      %legend(p,LegendNames,'Location','NorthEastOutside')
      legend(p,LegendNames)
    end
  end
end

if (DELTA)
  xdelt = xout(1,:)-xout(NSTATES+1,:);
  plottool(1,'xdelt',12,'Time(sec)','X Wingtip Error(m)');
  plot(tout,xdelt,'k-','LineWidth',2)
  ydelt = xdelt;
  for ii = 1:length(xdelt)
      rparent = xout(1:3,ii);
      phi = xout(4,ii);
      theta = xout(5,ii);
      psi = xout(6,ii);
      TIBParent = R123(phi,theta,psi);
      parentwing = rparent + TIBParent*[0;1.02;0];
      rchild = xout((NSTATES+1):(NSTATES+3),ii);
      phi = xout(NSTATES+4,ii);
      theta = xout(NSTATES+5,ii);
      psi = xout(NSTATES+6,ii);
      TIBchild = R123(phi,theta,psi);
      childwing = rchild + TIBchild*[0;-1.02;0];
      ydelt(ii) = parentwing(2) - childwing(2);
  end
  plottool(1,'xdelt',12,'Time(sec)','Y Wingtip Error(m)');
  plot(tout,ydelt,'k-','LineWidth',2)
  if DUAL
      plottool(1,'Dualplot',12,'Time(sec)');
      plot(tout,-100.*ydelt,'k-','LineWidth',2);
      data = dlmread('source/Out_Files/Meta_Force.OUT');
      forceout = data(:,2:end);
      plot(tout,-forceout(:,8),'k--','LineWidth',2);
      legend('Y Wingtip Error(cm)','Contact Force(N)')
      plottool(1,'MagnetForce',12,'Time','Y Magnet Force(N)');
      plot(tout,forceout(:,14),'k-','LineWidth',2)
  end
end



