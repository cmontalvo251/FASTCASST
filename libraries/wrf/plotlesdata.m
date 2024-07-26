%%This code will plot the terrain and spatial cross sections as
%well as plot u,v, and w at a specific location as a function of time
clear
clc
close all

%tic

%%Front End
UVWfrontend

%%ADMIN FLAGS

TERRAIN = 0;
PLOTT = 0; %%plot single points as a function of time
PLOT1 = 0; %%plot single points along a line in space
PLOT2 = 0; %%plot cross sections at a given instant in time
PLOTARROWS = 0; %plot arrows
DRYDEN = 0; %%Add in dryden turbulence
zflag = 0; %%0 = relative to ground, 1 = relative to sea-level

%%%%%%DRYDEN TURBULENCE PARAMETERS%%%%%%%%%%%%%%

V = 118; %%a/c flight speed in ft/s
timestep = 0.5;
ugust = 0; %%initialize gust
vgust = 0;
wgust = 0;
if DRYDEN
  %Find W20 = mean lateral wind component at 20 feet
  W20 = 0;
  for ii = 1:length(xcoord)
    for jj = 1:length(ycoord)
      uvw = uvwout(xcoord(ii),ycoord(jj),zcoord(1),0,dataloc,zflag);
      W20 = W20 + sqrt(uvw(1)^2 + uvw(2)^2);
    end
  end
  W20 = W20*3.28/(length(xcoord)^2);
  W20 = 2;
  sigmaw = 0.1*W20;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%Import Terrain Height
height = dlmread([dataloc,'THeight.txt']);

%%Create X and Y mesh
[xx2,yy2] = meshgrid(xcoord,ycoord);
[xx3,zz3] = meshgrid(xcoord,zcoord);
meshview = [-27,30];

%%%%%%%%%%%%%%%%%%%%%

if TERRAIN
  plottool(1,'MESH',12,'x (m)','y (m)','z(m)',['Terrain Data'],meshview)
  meshr(xx2,yy2,height,1)
end


%%%%%%%%%%%%%%Plot Single points as a function of time%%%%%%%%%%%%%%%%%

if PLOTT
  xstar = 0;
  ystar = 0;
  zstar = 10; %meters
  tstart = 0;
  tend = 3;
  tstartloc = find(tcoord >= tstart,1);
  tendloc = find(tcoord >= tend,1);
  tlength = length(tstart:1:tend);
  uvw = zeros(3,tlength);
  for ii = tstartloc:tendloc
    uvw(:,ii) = uvwout(xstar,ystar,zstar,tcoord(ii),dataloc,zflag);
    disp(['Time = ',num2str(tcoord(ii))])
  end
  if DRYDEN
    tdryden = tstart:timestep:tend;
    uvwgust = zeros(3,length(tdryden));
    uvwinterp = uvwgust;
    uvwdryden = uvwgust;
    uvwinterp(:,1) = uvw(:,1);
    uvwdryden(:,1) = uvw(:,1);
    for ii = 2:length(tdryden)
      h = zstar/3.28;
      Lw = h;
      Lv = h/((0.177+0.000823*h)^1.2);
      Lu = Lv;
      sigmav = sigmaw/((0.177+0.000823*h)^0.4);
      sigmau = sigmav;
      eta1 = -1 + 2*rand;
      eta2 = -1 + 2*rand;
      eta3 = -1 + 2*rand;
      ugust = (1-V*timestep/Lu)*ugust + sqrt(2*timestep*(V/Lu))*sigmau*eta1;
      vgust = (1-V*timestep/Lv)*vgust + sqrt(2*timestep*(V/Lv))*sigmav*eta2;
      wgust = (1-V*timestep/Lw)*wgust + sqrt(2*timestep*(V/Lw))*sigmaw*eta3;
      uvwgust(:,ii) = [ugust;vgust;wgust]./3.28;
      uvwinterp(1,ii) = interp1(tstart:1:tend,uvw(1,:),tdryden(ii));
      uvwinterp(2,ii) = interp1(tstart:1:tend,uvw(2,:),tdryden(ii));
      uvwinterp(3,ii) = interp1(tstart:1:tend,uvw(3,:),tdryden(ii));
    end
    uvwdryden = uvwinterp + uvwgust;
  end
  
  %%Plot Routine
  
  titlename = ['x = ',num2str(xstar),' y = ',num2str(ystar),' z =',num2str(zstar),' m'];
  subtool(3,1,tstart:tend,uvw,'UVW',{'U m/s','V m/s','W m/s'},'Time(sec)',2,'bo-',titlename)
  if DRYDEN
    subtool(3,1,tdryden,uvwdryden,'UVW',{'U m/s','V m/s','W m/s'},'Time(sec)',2,'b-',['Dryden ',titlename])
    plottool(1,'U',12,'Time(sec)','U(m/s)',titlename)
    plot(tdryden,uvwinterp(1,:),'b-')
    plot(tdryden,uvwgust(1,:),'r-')
    plot(tdryden,uvwdryden(1,:),'b-','LineWidth',2)
    plottool(1,'V',12,'Time(sec)','V(m/s)',titlename)
    plot(tdryden,uvwinterp(2,:),'b-')
    plot(tdryden,uvwgust(2,:),'r-')
    plot(tdryden,uvwdryden(2,:),'b-','LineWidth',2)
    plottool(1,'W',12,'Time(sec)','W(m/s)',titlename)
    plot(tdryden,uvwinterp(3,:),'b-')
    plot(tdryden,uvwgust(3,:),'r-')
    plot(tdryden,uvwdryden(3,:),'b-','LineWidth',2)
  end
end

%%%%%%%%%%%Plot Cross Sections at a given instant in time%%%%%%

if PLOT1
  %%pick xz, yz, or xy cross section
  independent = 'x';
  location1 = 0 %%pick the other two dependent variables
  location2 = 200
  time = 1;      %%finally pick the time you'd like to look at
  UPLOT = 1;
  VPLOT = 1;
  WPLOT = 1;
  if strcmp(independent,'x')
    dim = dimX;
  elseif strcmp(independent,'y')
    dim = dimY;
  elseif strcmp(independent,'z')
    dim = dimZ;
  end
  Umap = zeros(dim,1);
  Vmap = Umap;
  Wmap = Umap;
  for ll = 1:length(time)
    for ii = 1:dim
      if strcmp(independent,'x')
	uvw = uvwout(xcoord(ii),location1,location2,time(ll),dataloc,zflag);
      elseif strcmp(independent,'y')
	uvw = uvwout(location1,ycoord(ii),location2,time(ll),dataloc,zflag);
      elseif strcmp(independent,'z')
	uvw = uvwout(location1,location2,zcoord(ii),time(ll),dataloc,zflag);
      end
      Umap(ii) = uvw(1);
      Vmap(ii) = uvw(2);
      Wmap(ii) = uvw(3);
    end
    if ll == 1
      flag = 1;
    else
      flag = 0;
    end
    if strcmp(independent,'x')
      rflag = 1;
      if UPLOT
	plottool(flag,'LINE',12,'x (m)','u(m/s)')
	plot(xcoord,Umap)
      end
      if VPLOT
	plottool(flag,'LINE',12,'x (m)','v(m/s)')
	plot(xcoord,Vmap)
      end
      if WPLOT
	plottool(flag,'LINE',12,'x (m)','w(m/s)')
	plot(xcoord,Wmap)
      end
      %M(ll) = getframe;
    elseif strcmp(independent,'xz')
      rflag = 0;
      if UPLOT
	plottool(flag,'LINE',12,'x (m)','z (m)','u(m/s)',['XZ Plane @ y = ',num2str(location),' and t = ',num2str(time(ll))],meshview)
	meshr(xx3,zz3,Umap,rflag)
      end
      if VPLOT
	plottool(flag,'LINE',12,'x (m)','z (m)','v(m/s)',['XZ Plane @ y = ',num2str(location),' and t = ',num2str(time(ll))],meshview)
	meshr(xx3,zz3,Vmap,rflag)
      end
      if WPLOT
	plottool(flag,'LINE',12,'x (m)','z (m)','w(m/s)',['XZ Plane @ y = ',num2str(location),' and t = ',num2str(time(ll))],meshview)
	meshr(xx3,zz3,Wmap,rflag)
      end
    elseif strcmp(independent,'yz')
      rflag = 0;
      if UPLOT
	plottool(flag,'LINE',12,'y (m)','z (m)','u(m/s)',['YZ Plane @ x = ',num2str(location),' and t = ',num2str(time(ll))],meshview)
	meshr(yytemp,zz3,Umap,rflag)
      end
      if VPLOT
	plottool(flag,'LINE',12,'y (m)','z (m)','v(m/s)',['YZ Plane @ x = ',num2str(location),' and t = ',num2str(time(ll))],meshview)
	meshr(yytemp,zz3,Vmap,rflag)
      end
      if WPLOT
	plottool(flag,'LINE',12,'y (m)','z (m)','w(m/s)',['YZ Plane @ x = ',num2str(location),' and t = ',num2str(time(ll))],meshview)
	meshr(yytemp,zz3,Wmap,rflag)
      end
    end
    %pause(0.01)
  end
end


%%%%%%%%%%%Plot Cross Sections at a given instant in time%%%%%%

if PLOT2
  name = '0000';
  %%pick xz, yz, or xy cross section
  plane = 'xy';
  location = 200; %%pick the plane then pick the independent variable for location
  time = 0;      %%finally pick the time you'd like to look at
  UPLOT = 1;
  VPLOT = 1;
  WPLOT = 1;
  PPAUSE = 0;
  if strcmp(plane,'xy')
    dim1 = dimX;
    dim2 = dimY;
  elseif strcmp(plane,'xz')
    dim1 = dimX;
    dim2 = dimZ;
  elseif strcmp(plane,'yz')
    dim1 = dimY;
    dim2 = dimZ;
  end
  Umap = zeros(dim2,dim1);
  Vmap = Umap;
  Wmap = Umap;
  for mm = 1:length(time)
    for ll = 1:length(location)
    for ii = 1:dim2
      for jj = 1:dim1
	if strcmp(plane,'xy')
	  uvw = uvwout(xx2(ii,jj),yy2(ii,jj),location(ll),time(mm),dataloc,zflag);
	elseif strcmp(plane,'xz')
	  uvw = uvwout(xx3(ii,jj),location(ll),zz3(ii,jj),time(mm),dataloc,zflag);
	elseif strcmp(plane,'yz')
	  yytemp = yy2';
	  uvw = uvwout(location(ll),yytemp(ii,jj),zz3(ii,jj),time(mm),dataloc,zflag);
	end
	Umap(ii,jj) = uvw(1);
	Vmap(ii,jj) = uvw(2);
	Wmap(ii,jj) = uvw(3);
      end
    end
    if ll == 1
      flag = 1;
    else
      flag = 0;
    end
    if strcmp(plane,'xy')
      rflag = 1;
      if UPLOT
	plottool(flag,'MESH',12,'x (m)','y (m)','u(m/s)',['XY Plane @ z = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	meshr(xx2,yy2,Umap,rflag)
      end
      if VPLOT
	plottool(flag,'MESH',12,'x (m)','y (m)','v(m/s)',['XY Plane @ z = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	meshr(xx2,yy2,Vmap,rflag)
      end
      if WPLOT
	plottool(flag,'MESH',12,'x (m)','y (m)','w(m/s)',['XY Plane @ z = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	surfr(xx2,yy2,Wmap,rflag)
      end
      %M(ll) = getframe;
    elseif strcmp(plane,'xz')
      rflag = 0;
      if UPLOT
	plottool(flag,'MESH',12,'x (m)','z (m)','u(m/s)',['XZ Plane @ y = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	meshr(xx3,zz3,Umap,rflag)
      end
      if VPLOT
	plottool(flag,'MESH',12,'x (m)','z (m)','v(m/s)',['XZ Plane @ y = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	meshr(xx3,zz3,Vmap,rflag)
      end
      if WPLOT
	plottool(flag,'MESH',12,'x (m)','z (m)','w(m/s)',['XZ Plane @ y = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	surfr(xx3,zz3,Wmap,rflag)
      end
    elseif strcmp(plane,'yz')
      rflag = 0;
      if UPLOT
	plottool(flag,'MESH',12,'y (m)','z (m)','u(m/s)',['YZ Plane @ x = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	meshr(yytemp,zz3,Umap,rflag)
      end
      if VPLOT
	plottool(flag,'MESH',12,'y (m)','z (m)','v(m/s)',['YZ Plane @ x = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	meshr(yytemp,zz3,Vmap,rflag)
      end
      if WPLOT
	plottool(flag,'MESH',12,'y (m)','z (m)','w(m/s)',['YZ Plane @ x = ',num2str(location(ll)),' and t = ',num2str(time(mm))],meshview);
	surfr(yytemp,zz3,Wmap,rflag)
      end
    end
    meanW(ll) = mean(mean(Wmap));
    meanU(ll) = mean(mean(Umap));
    meanV(ll) = mean(mean(Vmap));
    maxW(ll) = max(max(Wmap));
    maxU(ll) = max(max(Umap));
    maxV(ll) = max(max(Vmap));
    stdU(ll) = std(unwrapmatrix(Umap));
    stdV(ll) = std(unwrapmatrix(Vmap));
    stdW(ll) = std(unwrapmatrix(Wmap));
    if PPAUSE
      %pause(0.01)
      %pause
      name = addition(name,'0001');
      saveas(gcf,name,'png');
      close all
    end
  end
  end
end


%%%%%%%%%%%

if PLOTARROWS
  %This will plot an xz cross section and plot arrows instead of a
  %mesh plot
  ystar = 0; %%pick cross section
  time = 0;
  zflag = 0;
  plottool(1,'Arrows',12,'x (m)','z (m)','','',[],'ylim',[0 ztop])
  pause
  %%get data of u and w
  Wmap = zeros(maxDim,maxDim);
  Umap = Wmap;
  for ll = 1:length(time)
    tstar = time(ll);
    for ii = 1:maxDim
      for jj = 1:maxDim
	uvw = uvwout(xx(ii,jj),ystar,zz(ii,jj),tstar,dataloc,zflag);
	Umap(ii,jj) = uvw(1);
	Wmap(ii,jj) = uvw(3);
      end
    end
    curheight = height(maxDim-markY,:);
    plottool(0,'Arrows',12,'x (m)','z (m)','','',[],'ylim',[0 ztop])
    quiver(xx,zz+ones(maxDim,1)*curheight,Umap,Wmap)
    %quiver(xx,zz,Umap,Wmap)
    plot(xcoord,height(maxDim-markY,:),'k-','LineWidth',3)
    pause(0.01)
  end
  %This will plot an yz cross section and plot arrows instead of a
  xstar = -47.5; %%pick cross section
  tstar = 0;
  plottool(1,'Arrows',12,'y (m)','z (m)','','',[],'ylim',[0 ztop])
  %%get data of v and w
  Wmap = zeros(maxDim,maxDim);
  Vmap = Wmap;
  yytemp = yy';
  for ii = 1:maxDim
    for jj = 1:maxDim
      uvw = uvwout(xstar,yytemp(ii,jj),zz(ii,jj),tstar,dataloc,zflag);
      Vmap(ii,jj) = uvw(2);
      Wmap(ii,jj) = uvw(3);
    end
  end
  curheight = height(:,markX)';
  quiver(yytemp,zz+ones(maxDim,1)*curheight,Vmap,Wmap)
  %quiver(yytemp,zz,Vmap,Wmap)
  plot(ycoord,height(:,markX),'k-','LineWidth',3)
end

fclose all;

% plottool(1,'Variation',12,'Altitude(m)','Standard Deviation(m/s)')
% alt = 0:100:1000;
% plot(alt,stdU,'k-','LineWidth',2)
% plot(alt,stdV,'k--','LineWidth',2)
% plot(alt,stdW,'k-.','LineWidth',2)
% legend('U','V','W')
%toc

%%Grab rays 
V = 20;
x = 0;
y = 0;
time = 0:100;
heading = linspace(0,2*pi,33);
uvwvec = zeros(3,length(time),length(heading));
headingvec = zeros(length(time),length(heading));
stdU = zeros(length(heading),1);
stdV = zeros(length(heading),1);
stdW = zeros(length(heading),1);

% U0 = dlmread([dataloc,'Uturb.txt']);
% V0 = dlmread([dataloc,'Vturb.txt']);
% W0 = dlmread([dataloc,'Wturb.txt']);

for ii = 1:length(heading)
  heading(ii)
  TIB = R123(0,0,heading(ii));
  x = 0;
  y = 0;
  for jj = 1:length(time)
    x = x + V*cos(heading(ii));
    y = y + V*sin(heading(ii));
    if (x > xcoord(end))
      x = x - xcoord(end) + xcoord(1);
    end
    if (x < xcoord(1))
      x = x - xcoord(1) + xcoord(end);
    end
    if (y > ycoord(end))
      y = y - ycoord(end) + ycoord(1);
    end
    if (y < ycoord(1))
      y = y - ycoord(1) + ycoord(end);
    end
    uvwInertial = uvwout(x,y,200,0,dataloc,zflag);
    %%Rotate to body frame
    uvwBody = TIB'*uvwInertial;
    uvw = uvwBody;
    uvwvec(:,jj,ii) = uvw;
    headingvec(jj,ii) = atan2(uvw(2),uvw(1));
  end
  a = uvwvec(:,:,ii);
  stdU(ii) = std(a(1,:));
  stdV(ii) = std(a(2,:));
  stdW(ii) = std(a(3,:));
  meanU(ii) = mean(a(1,:));
  meanV(ii) = mean(a(2,:));
  meanW(ii) = mean(a(3,:));
  %plot(time,a)
  %legend('u','v','w')
end
heading = heading.*180/pi;
plot(heading,stdU,'b-')
hold on
plot(heading,stdV,'r-')
plot(heading,stdW,'g-')
figure()
plot(heading,meanU,'b-')
hold on
plot(heading,meanV,'r-')
plot(heading,meanW,'g-')
figure()
mesh(headingvec)

% Copyright - Carlos Montalvo 2015
% You may freely distribute this file but please keep my name in here
% as the original owner
