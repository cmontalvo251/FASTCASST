purge

scale = 0.01;
add = 225.45;
Az = -58;
El = 40;
[fig1,ax1] = plottool(1,'U',12,'Latitude(deg)','Longitude(deg)','Speed(m/s)',num2str(0),[Az,El]);
[fig2,ax2] = plottool(1,'V',12,'Latitude(deg)','Longitude(deg)','Speed(m/s)',num2str(0),[Az,El]);

pause
for ii = 0
  time = ii;

  %%import
  eval(['ufile = ''Compiled_Data/U',num2str(time),'.txt'';'])
  eval(['vfile = ''Compiled_Data/V',num2str(time),'.txt'';'])

  udata = dlmread(ufile);
  vdata = dlmread(vfile);

  %%Convert
  udata = (udata.*scale)+add;
  vdata = (vdata.*scale)+add;
  [r,c] = size(udata);

  %PLot
  lat = linspace(90,-90,r);
  lon = linspace(0,360,c);
  llon = zeros(r,c);
  llat = zeros(r,c);
  
  for idx = 1:length(lat)
    for jdx = 1:length(lon)
      llon(idx,jdx) = lon(jdx);
      llat(idx,jdx) = lat(idx);
    end
  end

  mesh(ax1,llat,llon,udata)
  title(ax1,['Day = ',num2str(time)])
  mesh(ax2,llat,llon,vdata)
  title(ax2,['Day = ',num2str(time)])
  
  %pause(0.01)

  %cla(ax1)
  %cla(ax2)
    
end
