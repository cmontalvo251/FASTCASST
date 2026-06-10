purge

fid = fopen('Tracking_Performance.txt');

header = 1;

numheadings = 20;

f = 1/numheadings;

meanXvec = [];
minXvec = [];
maxXvec = [];
stdXvec = [];
windlevels = [];

while header ~= -1
    for ii = 1:numheadings
        header = fgetl(fid);
        if header ~= -1
            header = str2num(header);
            meanX = f*str2num(fgetl(fid));
            minX = f*str2num(fgetl(fid));
            maxX = f*str2num(fgetl(fid));
            stdX = f*str2num(fgetl(fid));
            if ii == 1
                meanXvec = [meanXvec;meanX];
                minXvec = [minXvec;minX];
                maxXvec = [maxXvec;maxX];
                stdXvec = [stdXvec;stdX];
                windlevels = [windlevels;header(2)];
            else
                meanXvec(end,:) = meanXvec(end,:) + meanX;
                minXvec(end,:) = minXvec(end,:) + minX;
                maxXvec(end,:) = maxXvec(end,:) + maxX;
                stdXvec(end,:) = stdXvec(end,:) + stdX;
            end
        end
    end
end
%LegendNames = {'Mean','Min','Max','Stdev'};
%LegendNames = {'Mean','Max'};
%LegendNames = {'Mean','Max'};
Names = {'X','Y','Z','Phi','Theta','Psi','U','V','W','P','Q','R','zint','xint','yint'};
dim1 = 'm';
ylabels = {['x(',dim1,')'],['y(',dim1,')'],['z(',dim1,')'],'\phi(deg)','\theta(deg)','\psi(deg)',['u(',dim1,'/s)'],['v(',dim1,'/s)'],['w(',dim1,'/s)'],'p(rad/s)','q(rad/s)','r(rad/s)','zint','xint','yint'};
units = 1;
LineWidth = 2;
colors = {'b','r','g','m','c','k'};
linetype = {'-','--','-.','-','--','-.'};
order = [4,3,3,2,3];
%%Normalize 1-3
xyz = sqrt(meanXvec(:,1).^2 + meanXvec(:,2).^2 + meanXvec(:,3).^2);
for ii = 1
  if ii <= 15
    if ii >= 4 && ii <= 6
      factor = 180/pi;
    else
      factor = 1;
    end
    h1 = plottool(1,'Tracking Error(m)',12,'Maximum Windspeed(m/s)','Tracking Error(m)');
    % [x,mx] = ismooth(windlevels,factor.*maxXvec(:,ii),11,order(ii));
    % p(1) = plot(windlevels,factor.*maxXvec(:,ii),'ks','LineWidth',LineWidth);
    % p(1) = plot(x,mx,'k-','LineWidth',LineWidth);
    % [x,mx] = ismooth(windlevels,factor.*minXvec(:,ii),11,order(ii));
    % p(2) = plot(windlevels,factor.*minXvec(:,ii),'kd','LineWidth',LineWidth);
    % p(2) = plot(x,mx,'k--','LineWidth',LineWidth);
    % [x,mx] = ismooth(windlevels,factor.*meanXvec(:,ii),11,order(ii));
    % p(1) = plot(windlevels,factor.*meanXvec(:,ii),'ks','LineWidth',LineWidth);
    % p(1) = plot(x,mx,'k-','LineWidth',LineWidth);
    wl = windlevels.*4;
    [x,mx] = ismooth(wl(1:26),factor.*xyz(1:26),30,order(ii));
    p(1) = plot(wl,factor.*xyz,'k-','LineWidth',LineWidth);
    %p(1) = plot(x,mx,'k-','LineWidth',LineWidth);
    %p(1) = plot(windlevels,factor.*minXvec(:,ii),'r-','LineWidth',LineWidth);
    %p(2) = plot(windlevels,factor.*maxXvec(:,ii),'g-','LineWidth',LineWidth);
    %p(4) =
    %plot(windlevels,factor.*stdXvec(:,ii),'k-','LineWidth',LineWidth);
    %legend(p,LegendNames,'Location','NorthEastOutside')
    %xlim([0 1.75])
    %legend('Data Points','Polynomial Fit')
    xlim([0 10])
  end
end