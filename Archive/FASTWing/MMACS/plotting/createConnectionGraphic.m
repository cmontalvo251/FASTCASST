close all

%%%%%STANDALONE%%%%%%%%%%
TYPE = 1;
STANDALONE = 0;
if STANDALONE 

  fname = 'plotting/MATS/DeltaX.mat';
  ts = 0.03;
  eval(['load ' fname]);
  SIMPLE = 1;
  TYPE = 0; %%0 = Follow both, 1 = zoom out, 2 = follow master only
end

%%%%%%%%%%%%%%%%%%%%%%%%%%


OBJECT = 0; %0=plane,1=cubes

if ts ~= timestep
  %%Find occurence of first inf or -inf
  [r,c] = size(xout);
  locnan = c;
  for ii = 1:r
    loc1 = find(isnan(xout(r,:)),1);
    if loc1 < locnan
      locnan = loc1;
    end
  end
  col = locnan-1;
  xnew = xout(:,1:locnan-1);
  tnew = tout(1:locnan-1);
  new_t = tnew(1):ts:tnew(end);
  err = 1;
  while err
    try
      disp('trying..')
      new_x = interp1(tnew(1:col),xnew(:,1:col)',new_t);
      err = 0;
    catch me
      err = 1;
      disp('error found')
      disp(me.message)
      col = col - 1
    end
  end
  disp('Interpolation Succeeded')
  new_x = new_x';
else
  new_t = tout;
  new_x = xout;
end

%%Truncate new_x
t = new_t;
tstart = 0;
tend = 21;
istart = find(t>tstart,1);
iend = find(t>tend,1);
skip = 10;
new_x = new_x(:,istart:iend);
t = t(istart:iend);

x = [];y=[];z=[];
for ii = 1:num_ac
  s = (ii-1)*(NSTATES)+1;
  x = [x;new_x(s,:)];
  y = [y;new_x(s+1,:)];
  z = [z;new_x(s+2,:)];
end
del = 0.5;
if num_ac == 1
  xmin = min(x)-del;
  xmax = max(x)+del;
  ymin = min(y)-del;
  ymax = max(y)+del;
  zmin = min(z)-del;
  zmax = max(z)+del;
else
  xmin = min(min(x))-del;
  xmax = max(max(x))+del;
  ymin = min(min(y))-del;
  ymax = max(max(y))+del;
  zmin = min(min(z))-del;
  zmax = max(max(z))+del;
end
%%Open ifiles
fid = fopen('source/Meta.ifiles');
TIMEFILE = fgetl(fid);
MASSFILE = fgetl(fid);
AEROFILE = fgetl(fid);
AERO = importdata(AEROFILE);
wingtext = AERO{37};
wingnum = wingtext(1:find(wingtext=='!')-1);
wingspan = str2num(wingnum);
wingspan = [wingspan;wingspan];
color = [.1 .1 .1];
color1 = [.8 .1 .1];
% import Solidworks STL files
[xf,yf,zf]=stlread('Fuselage.STL');
[xL,yL,zL]=stlread('LWing.STL');
[xR,yR,zR]=stlread('RWing.STL');
[xp,yp,zp]=stlread('Prop.STL');

xyzf = [xf;yf;zf];
xyzL = [xL;yL;zL];
xyzR = [xR;yR;zR];
xyzp = [xp;yp;zp];
%%Create Colormap
dg = [0 150 0]./255;
cht = [127 255 0]./255;
rvec = dg(1):cht(1)';
gvec = dg(2):cht(2)';
bvec = dg(3):cht(3)';
%%Create Terrain
%limits(5:6) = [target(3) target(3)]+0.2;
limits = [xmin-10 xmax ymin-10 ymax+10 0 0];
limits(5:6) = [3 3];
N = 50;
h1 = plottool(1,'Plane',12,'x(ft)','y(ft)','z(ft)');
pause
skip = 1;
origin = zeros(6,1);
[r,c] = size(new_x);
xp = zeros(c,1);
yp = xp;
zp = xp;
for i = 1:c
  %hold off
  %cla;
  for ii = 1:2
    num = (ii-1)*NSTATES+1;
    state = new_x(num:num+5,i)./2;
    if ii == 1
      origin(1:3) = state(1:3);
    else
      new = state(1:3);
      plotstate = state-origin;
      OK = 0;
      if norm(new-old) > 5
        old(1:3) = new;
        OK = 1;
      end    
      if OK && plotstate(1) <= 0
	draw_plane_fancy(state-origin,xyzf,xyzL,xyzR,xyzp,wingspan(ii),t(i))
	xp(i) = plotstate(1);
	yp(i) = plotstate(2);
	zp(i) = plotstate(3);
      end
    end
  end
  title(num2str(t(i)))
  hold on
  reverse(['y','z'])
  xlabel('x')
  ylabel('y')
  zlabel('z')
  %axis([xmin xmax ymin ymax zmin zmax])
  axis off
  axis equal
  
  grid on
  view(Az,El)
  drawnow
end
xp(xp==0)=[];
yp(yp==0)=[];
zp(zp==0)=[];
plot3(xp,yp,zp,'b--','LineWidth',1)
draw_plane_fancy([0;0;0;0;0;0],xyzf,xyzL,xyzR,xyzp,wingspan(1),1)
Sun = light('Position',-1000.*[0 0 2],'Style','infinite');
axis equal