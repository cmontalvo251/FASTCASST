close all

%%Plot Routine
xoutAll = {};
uoutAll = {};
toutAll = {};
Legends = {};
if exist('xout6','var')
  xoutAll = [xoutAll;xout6];
  uoutAll = [uoutAll;uout6];
  toutAll = [toutAll;tout6];
  Legends = [Legends;'Single Com Pts'];
end
if exist('xoutSM','var')
  xoutAll = [xoutAll;xoutSM];
  uoutAll = [uoutAll;uoutSM];
  toutAll = [toutAll;toutSM];
  Legends = [Legends;'Multiple Com Pts'];
end
if exist('xoutLTI','var')
  xoutAll = [xoutAll;xoutLTI];
  uoutAll = [uoutAll;uoutLTI];
  toutAll = [toutAll;toutLTI];
  Legends = [Legends;'LTI'];
end
if exist('xoutD','var')
  xoutAll = [xoutAll;xoutD];
  uoutAll = [uoutAll;uoutD];
  toutAll = [toutAll;toutD];
  Legends = [Legends;'Discrete Model'];
end
colors = {'b-','r-','g-'};

if STATES
  l = 'm';
  f = 'N';
  units = {['x(',l,')'],['y(',l,')'],['z(',l,')'],'\phi(deg)','\theta(deg)','\psi(deg)',['u(',l,'/s)'],['v(',l,'/s)'],['w(',l,'/s)'],'p(rad/s)','q(rad/s)','r(rad/s)',['Fx(',f,')'],['Fy(',f,')'],['Fz(',f,')'],['L(',f,'-',l,')'],['M(',f,'-',l,')'],['N(',f,'-',l,')']};
  for ii = 1:12
    h1 = plottool(1,units{ii},12,'Time(sec)',units{ii});
    if ii > 3 && ii < 7
      factor = 180/pi;
    else
      factor = 1;
    end 
    for jj = 1:length(xoutAll)
      xout = xoutAll{jj};
      tout = toutAll{jj};
      p1=plot(tout,xout(ii,:).*factor,colors{jj},'LineWidth',2);
    end
    legend(Legends)
  end
end

if CONTROLS
  units = {'da','dr','de','dT'};
  for ii = 1:4
    h1 = plottool(1,units{ii},12,'Time(sec)',units{ii});
    factor = 1;
    for jj = 1:length(uoutAll)
      uout = uoutAll{jj};
      tout = toutAll{jj};
      p1=plot(tout,uout(ii,:).*factor,colors{jj},'LineWidth',2);
    end
    legend(Legends)
  end
end


% figure()
% plot(tout6,XYZvec)
% hold on
% plot(tout6,XYZPvec)
% legend('X','Y','Z')

% figure()
% plot(tout6,LMNvec)
% hold on
% plot(tout6,LMNPvec)
% legend('L','M','N')
