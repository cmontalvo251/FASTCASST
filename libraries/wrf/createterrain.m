function [xx,yy,zz] = createterrain(limits,N)

xmin = limits(1);
xmax = limits(2);
ymin = limits(3);
ymax = limits(4);
zmin = limits(5);
zmax = limits(6);

xvec = linspace(xmin,xmax,N);
yvec = linspace(ymin,ymax,N);
[xx,yy] = meshgrid(xvec,yvec);
zz = 0.*xx;
%%Create terrain
for ii = 1:N
  for jj = 1:N
    x = xx(ii,jj);
    y = yy(ii,jj);
    zz(ii,jj) = zmax-0.1+0.8*(0.05*cos(7*xx(ii,jj)) + 0.05*cos(7*yy(ii,jj)-xx(ii,jj)) + 0.05*cos(11*xx(ii,jj)) + 0.05*cos(11*yy(ii,jj)));
  end
end





% Copyright - Carlos Montalvo 2015
% You may freely distribute this file but please keep my name in here
% as the original owner
