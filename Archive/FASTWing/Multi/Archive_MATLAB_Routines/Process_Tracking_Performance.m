purge

fid = fopen('Tracking_Performance.txt');

header = [1 1];

numRuns = 100;
numrows = 3;
numcols = 3;

f = numRuns;

meanXvec = zeros(numrows,numcols,12); %%row,col
minXvec = zeros(numrows,numcols,12);
maxXvec = zeros(numrows,numcols,12);
stdXvec =zeros(numrows,numcols,12);

for nn = 1:numrows
  nn
  for mm = 1:numcols
    mm
    f = numRuns;
    for ii = 1:numRuns
      header = fgetl(fid);
      if header(1) ~= -1
	header = str2num(header);
	row = header(1);
	col = header(2);
	meanX = str2num(fgetl(fid));
	minX = str2num(fgetl(fid));
	maxX = str2num(fgetl(fid));
	stdX = str2num(fgetl(fid));
	%%%Check for Nans
	if isnan(meanX(1,1))
	  f = f - 1;
	else
	  for ll = 1:12
	    meanXvec(nn,mm,ll) = meanXvec(nn,mm,ll) + meanX(ll);
	    minXvec(nn,mm,ll) = minXvec(nn,mm,ll) + minX(ll);
	    maxXvec(nn,mm,ll) = maxXvec(nn,mm,ll) + maxX(ll);
	    stdXvec(nn,mm,ll) = stdXvec(nn,mm,ll) + stdX(ll);
	  end
	end %%isnan
      end %%header
    end %%numRuns
    meanXvec(nn,mm,:) = meanXvec(nn,mm,:)./f;
    minXvec(nn,mm,:) = minXvec(nn,mm,:)./f;
    maxXvec(nn,mm,:) = maxXvec(nn,mm,:)./f;
    stdXvec(nn,mm,:) = stdXvec(nn,mm,:)./f;
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

%%%Edit 1 point
%meanXvec(4,4,3) = 45;

for ii = [2,3,4,5,6]
  if ii <= 15
    if ii >= 4 && ii <= 6
      factor = 180/pi;
    else
      factor = 1;
    end
    %%%%MEAN
    h1 = plottool(1,[Names{ii},'mean'],12,'Number of Columns','Number of Rows');
    %mesh(factor.*meanXvec(:,:,ii))
    [C,h] = contour(factor.*meanXvec(:,:,ii),'k','LineWidth',1);
    clabel(C,h,'FontSize',10)
    
    %%%%MIN
    % h1 = plottool(1,[Names{ii},'min'],12,'Number of Columns','Number of Rows');
    % [C,h] = contour(factor.*minXvec(:,:,ii),'k','LineWidth',1);
    % clabel(C,h,'FontSize',10)
    % mesh(factor.*minXvec(:,:,ii))
    
    %%%MAX
    % h1 = plottool(1,[Names{ii},'max'],12,'Number of Columns','Number of Rows');
    % mesh(factor.*maxXvec(:,:,ii))
    % [C,h] = contour(factor.*maxXvec(:,:,ii),'k','LineWidth',1);
    % clabel(C,h,'FontSize',10)
    
    %%%%STD
    % h1 = plottool(1,[Names{ii},'std'],12,'Number of Columns','Number of Rows');
    % [C,h] = contour(factor.*stdXvec(:,:,ii),'k','LineWidth',1);
    % clabel(C,h,'FontSize',10)
    % mesh(stdXvec(:,:,ii))
  end
end