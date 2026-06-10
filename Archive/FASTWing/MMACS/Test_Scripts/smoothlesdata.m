%%This code will plot the terrain and spatial cross sections as
%well as plot u,v, and w at a specific location as a function of time
clear
clc
close all

%tic

%%Front End
UVWfrontend

PLOT2 = 1; %%plot cross sections at a given instant in time
zflag = 0; %%0 = relative to ground, 1 = relative to sea-level

%%Create X and Y mesh
[xx2,yy2] = meshgrid(xcoord,ycoord);
[xx3,zz3] = meshgrid(xcoord,zcoord);
meshview = [-27,30];

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
        Nsmooth = 4;
        Umap = smoothquad(Umap,Nsmooth);
        Vmap = smoothquad(Vmap,Nsmooth);
        Wmap = smoothquad(Wmap,Nsmooth);
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
