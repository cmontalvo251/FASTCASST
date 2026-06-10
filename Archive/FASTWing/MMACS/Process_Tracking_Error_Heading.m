purge

fid = fopen('Tracking_Performance.txt');

header = 1;

numheadings = 20;
headings = linspace(0,2*pi,numheadings);
windlevels = 0:0.1:3;
numwindlevels = length(windlevels);
[ww,hh] = meshgrid(windlevels,headings);

meanXvec = [];
minXvec = [];
maxXvec = [];
stdXvec = [];

xmean = zeros(numheadings,numwindlevels);
ymean = xmean;
zmean = xmean;
phimean = xmean;
thetamean = xmean;

while header ~= -1
    for ii = 1:numwindlevels
        for jj = 1:numheadings
          header = fgetl(fid);
          if header ~= -1
            header = str2num(header);
            meanX = str2num(fgetl(fid));
            minX = str2num(fgetl(fid));
            maxX = str2num(fgetl(fid));
            stdX = str2num(fgetl(fid));
            xmean(jj,ii) = meanX(1);
            ymean(jj,ii) = meanX(2);
            zmean(jj,ii) = meanX(3);
            phimean(jj,ii) = meanX(4);
            thetamean(jj,ii) = meanX(5);
          end
        end
    end
end
plottool(1,'Xmean',12,'x','y','Xmean','',[-27,30]);
mesh(ww,hh,xmean)
