function [uvw] = uvwout(xstar,ystar,zstar,tstar,dataloc,zflag)
%function [uvw] = uvwout(xstar,ystar,zstar,tstar,dataloc,zflag)
%%This function will take in x,y,z(m),t(sec) and location and 
%return u,v,w(m/s). This uses a fast quad-linear interpolation
%so many globals must be defined. location is a string that
%contains the location of the data to be interpolated.
%DONT FORGET TO RUN UVWfrontend.m only once before you use
%this code
%Note when you plug in zstar this code will compute height above
%ground. So make sure you give this code the absolute height and
%this code will compute height above ground.
%%if zflag == 1 zstar is height above sea-level
%%if zlfag == 0 zstar is height above ground

global markX markY markZ markT parameters zcoord tcoord
global U0 Udt V0 Vdt W0 Wdt xcoord ycoord tlength tstr
global bounds boundflag terrain

uvw = [0;0;0];
stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
tinterp = 2;
%%Unwrap parameters
if length(parameters) == 7
  dimX = parameters(6);
  dimY = parameters(7);
  dimZ = parameters(5);
else
  dimZ = parameters(5);
  dimX = dimZ;
  dimY = dimZ;
end

%%Check X
if markX == dimX;
  markX = markX - 1;
end
if (xstar >= xcoord(markX)) && (xstar <= xcoord(markX+1))
  %%Your in between the markers so keep going
else
  %Find markX
  if xstar > xcoord(end)
    %use endpt
    markX = dimX;
    stepX = -1;
    extrapX = 1;
  elseif xstar < xcoord(1)
    %use starpt
    markX = 1;
    stepX = 1;
    extrapX = 1;    
  else
    markX = find(xcoord >= xstar,1)-1;
   if markX == dimX
      markX = markX - 1;
    elseif markX <= 0
        markX = 1;
    end
  end
end
%%Check Y
if markY == dimY;
  markY = markY - 1;
end
if (ystar >= ycoord(markY)) && (ystar <= ycoord(markY+1))
  %%Your in between the markers so keep going
else
  %Find markY
  if ystar > ycoord(end)
    %use endpt
    markY = dimY;
    stepY = -1;
    extrapY = 1;
  elseif ystar < ycoord(1)
    markY = 1;
    stepY = 1;
    extrapY = 1;
  else
    markY = find(ycoord >= ystar,1)-1;
    if markY == dimY
      markY = markY - 1;
    elseif markY <= 0
      markY = 1;
    end
  end
end
%%Check Z
if markZ == dimZ;
  markZ = markZ - 1;
end

%%if zflag == 1 zstar is height above sea-level
%%if zlfag == 0 zstar is height above ground
if zflag %zstar is height above sea-level
  %%We must convert to height above ground
  zground = interpsave2(xcoord,ycoord,terrain,xstar,ystar);
  zstar = zstar - zground;
  if zstar < 0
    return;
  end
else
  %zstar is height above ground so do nothing
end

if (zstar >= zcoord(markZ)) && (zstar <= zcoord(markZ+1))
  %%Your in between the markers so keep going
else
  %Find markZ
  if zstar > zcoord(end)
    %use endpt
    markZ = dimZ;
    stepZ = -1;
    extrapZ = 1;
  elseif zstar < zcoord(1)
    markZ = 1;
    stepZ = 1;
    extrapZ = 1;
  else
    markZ = find(zcoord >= zstar,1)-1;
    if markZ == dimZ
      markZ = markZ - 1;
    elseif markZ == 0
      markZ = 1;
    end
  end
end
%%Check T
if markT == tlength;
  markT = markT - 1;
end
if (tstar >= tcoord(markT)) && (tstar <= tcoord(markT+1))
  %%Your in between the markers so keep going
else
  %Find markT
  if tstar > tcoord(end)
    %use endpt
    markT = tlength;
    extrapT = 1;
  elseif tstar < tcoord(1)
    %use start pt
    markT = 1;
    extrapT = 1;
  else
    markT = find(tcoord >= tstar,1)-1;
    if markT == tlength
      markT = markT - 1;
    elseif markT == 0
      markT = 1;
    end
  end
  %%Import U,V,W maps since markT changed
  t0str = tstr{markT};
  U0name = [dataloc,'U',t0str,'.txt'];
  V0name = [dataloc,'V',t0str,'.txt'];
  W0name = [dataloc,'W',t0str,'.txt'];
  %%only import at markT
  U0 = importwind(U0name,dimX,dimY,dimZ);
  V0 = importwind(V0name,dimX,dimY,dimZ);
  W0 = importwind(W0name,dimX,dimY,dimZ);
  %U0 = U0(end:-1:1,end:-1:1,:);
  %V0 = V0(end:-1:1,end:-1:1,:);
  %W0 = W0(end:-1:1,end:-1:1,:);
  if extrapT
    tinterp = 1;
  else
    %%import markT + 1
    dtstr = tstr{markT+1};
    Udtname = [dataloc,'U',dtstr,'.txt'];
    Vdtname = [dataloc,'V',dtstr,'.txt'];
    Wdtname = [dataloc,'W',dtstr,'.txt'];
    Udt = importwind(Udtname,dimX,dimY,dimZ);
    Vdt = importwind(Vdtname,dimX,dimY,dimZ);
    Wdt = importwind(Wdtname,dimX,dimY,dimZ);
    %Udt = Udt(end:-1:1,end:-1:1,:);
    %Vdt = Vdt(end:-1:1,end:-1:1,:);
    %Wdt = Wdt(end:-1:1,end:-1:1,:);
  end
end


%%Interpolation Scheme
uvw = zeros(3,tinterp);
for tt = 1:tinterp 
  %Interpolate Spatially
    
  %%To start we have 8 discrete point (8 corners of a cube)
  xpts = [xcoord(markX) xcoord(markX+stepX)];
  ypts = [ycoord(markY) ycoord(markY+stepY)];
  zpts = [zcoord(markZ) zcoord(markZ+stepZ)];
  x1 = markX;x2 = markX+stepX;
  y1 = markY;y2 = (markY+stepY);
  z1 = markZ;z2 = markZ+stepZ;
  if tt == 1
    %%Use U0,V0,W0
    u = [U0(y1,x1,z1);U0(y1,x2,z1);U0(y2,x2,z1);U0(y2,x1,z1);U0(y1,x1,z2);U0(y1,x2,z2);U0(y2,x2,z2);U0(y2,x1,z2)];
    v = [V0(y1,x1,z1);V0(y1,x2,z1);V0(y2,x2,z1);V0(y2,x1,z1);V0(y1,x1,z2);V0(y1,x2,z2);V0(y2,x2,z2);V0(y2,x1,z2)];
    w = [W0(y1,x1,z1);W0(y1,x2,z1);W0(y2,x2,z1);W0(y2,x1,z1);W0(y1,x1,z2);W0(y1,x2,z2);W0(y2,x2,z2);W0(y2,x1,z2)];
  else
    %%Use Udt,Vdt,Wdt
    u = [Udt(y1,x1,z1);Udt(y1,x2,z1);Udt(y2,x2,z1);Udt(y2,x1,z1);Udt(y1,x1,z2);Udt(y1,x2,z2);Udt(y2,x2,z2);Udt(y2,x1,z2)];
    v = [Vdt(y1,x1,z1);Vdt(y1,x2,z1);Vdt(y2,x2,z1);Vdt(y2,x1,z1);Vdt(y1,x1,z2);Vdt(y1,x2,z2);Vdt(y2,x2,z2);Vdt(y2,x1,z2)];
    w = [Wdt(y1,x1,z1);Wdt(y1,x2,z1);Wdt(y2,x2,z1);Wdt(y2,x1,z1);Wdt(y1,x1,z2);Wdt(y1,x2,z2);Wdt(y2,x2,z2);Wdt(y2,x1,z2)];
  end
  
  
  %%%%%interpZ%%%%%%%%%%%%
  
  if extrapZ
    %%You don't need to interpolate on z and you can just use
    %%the values at markZ or z1
    zpts = zpts(1);
    u = u(1:4);
    v = v(1:4);
    w = w(1:4);
    bounds = 1;
  else
    %%Interpolate Between Z points(interpolate pts 1-4 and 5-8)
    %Pts 1,5 : 2,6 : 3,7 : 4,8
    coord1 = [1 2 3 4];
    coord2 = [5 6 7 8];
    uZ = [0;0;0;0];vZ = uZ;wZ = uZ;
    for ii = 1:4
      uslope = (u(coord2(ii))-u(coord1(ii)))/(zpts(2)-zpts(1));
      vslope = (v(coord2(ii))-v(coord1(ii)))/(zpts(2)-zpts(1));
      wslope = (w(coord2(ii))-w(coord1(ii)))/(zpts(2)-zpts(1));
      uZ(ii) = uslope*(zstar-zpts(1))+u(coord1(ii));
      vZ(ii) = vslope*(zstar-zpts(1))+v(coord1(ii));
      wZ(ii) = wslope*(zstar-zpts(1))+w(coord1(ii));
    end
    u = uZ;
    v = vZ;
    w = wZ;
    zpts = zstar;
  end
  %%%%%interpY%%%%%%%%%%%
  if extrapY
    %%You don't need to interpolate on y
    ypts = ypts(1);
    u = u(1:2);
    v = v(1:2);
    w = w(1:2);
    bounds = 1;
  else
    %%Interpolate between Y points(interpolate pts 1-2 and 3-4)
    %%Pts 1,4 : 2,3
    coord1 = [1 2];
    coord2 = [4 3];
    uY = [0;0];vY = uY;wY = uY;
    for ii = 1:2
      uslope = (u(coord2(ii))-u(coord1(ii)))/(ypts(2)-ypts(1));
      vslope = (v(coord2(ii))-v(coord1(ii)))/(ypts(2)-ypts(1));
      wslope = (w(coord2(ii))-w(coord1(ii)))/(ypts(2)-ypts(1));
      uY(ii) = uslope*(ystar-ypts(1))+u(coord1(ii));
      vY(ii) = vslope*(ystar-ypts(1))+v(coord1(ii));
      wY(ii) = wslope*(ystar-ypts(1))+w(coord1(ii));
    end
    u = uY;
    v = vY;
    w = wY;
    ypts = ystar;
  end

  %%%%interpX%%%%%%%%%%%%
  if extrapX
    %%You don't need to interpolate on x
    xpts = xpts(1);
    u = u(1);
    v = v(1);
    w = w(1);
    bounds = 1;
  else
    %%Interpolate between X points
    uslope = (u(2)-u(1))/(xpts(2)-xpts(1));
    vslope = (v(2)-v(1))/(xpts(2)-xpts(1));
    wslope = (w(2)-w(1))/(xpts(2)-xpts(1));
    u = uslope*(xstar-xpts(1))+u(1);
    v = vslope*(xstar-xpts(1))+v(1);
    w = wslope*(xstar-xpts(1))+w(1);
    xpts = xstar;
  end
  
  %%%%Save wind values%%%%%
  
  uvw(:,tt) = [u;v;w];
  
end

if extrapT
  %%Do nothing
else
  %%Interpolate on T
  tpts = [tcoord(markT) tcoord(markT+1)];
  u = uvw(1,:);
  v = uvw(2,:);
  w = uvw(3,:);
  uslope = (u(2)-u(1))/(tpts(2)-tpts(1));
  vslope = (v(2)-v(1))/(tpts(2)-tpts(1));
  wslope = (w(2)-w(1))/(tpts(2)-tpts(1));  
  u = uslope*(tstar-tpts(1))+u(1);
  v = vslope*(tstar-tpts(1))+v(1);
  w = wslope*(tstar-tpts(1))+w(1);
  uvw = [u;v;w];
end

if bounds && boundflag
  disp(['You went out of bounds at T = ',num2str(tstar),[' XYZ = ',num2str([xstar,ystar,zstar])]])
  boundflag = 0;
end



% Copyright - Carlos Montalvo 2015
% You may freely distribute this file but please keep my name in here
% as the original owner
