%%%%This File will take data outputted from savelesdata.ncl and 
%consolidate the data into 3 text files per timestep

disp('Running Consolidate')

%%%%%YOU ONLY NEED TO EDIT THE NEXT 5 LINES OF CODE
system('rm -r ~/Georgia_Tech/Grad_Research/WindMapper/Wind_Modeling/Simulation/Data/');
system('mkdir ~/Georgia_Tech/Grad_Research/WindMapper/Wind_Modeling/Simulation/Data/');
rawdataloc = '~/Georgia_Tech/Grad_Research/WindMapper/Wind_Modeling/Simulation/Raw_Data/';
simdataloc = '~/Georgia_Tech/Grad_Research/WindMapper/Wind_Modeling/Simulation/Data/';
namelist_loc = fopen('~/Georgia_Tech/Grad_Research/WindMapper/Wind_Modeling/WRFV3/test/em_les/namelist.input');

%eval(['addpath ',simdataloc])
%eval(['addpath ',rawdataloc])

%%Import TimeStamps

disp('Reading in Timestamps');
timestamps = dlmread([rawdataloc,'Times.txt']);

%%Import Z coordinates
z = dlmread([rawdataloc,'Z.txt']);
z = z(:,1);

%%Import Terrain Height

height = dlmread([rawdataloc,'TerrainHeight.txt']);

%%Import Namelist.input
inputs = fscanf(namelist_loc,'%s');
outputs = strread(inputs,'%s','delimiter',',');
dimX = outputs{26};
dimY = outputs{28};
dimZ = outputs{30};
dimZ = str2num(dimZ(find(dimZ=='=')+1:end))-1; %%km
dimY = str2num(dimY(find(dimY=='=')+1:end))-1; %%km
dimX = str2num(dimX(find(dimX=='=')+1:end))-1; %%km
dx = outputs{31};
dx = str2num(dx(find(dx=='=')+1:end)); %%m
dy = outputs{32};
dy = str2num(dy(find(dy=='=')+1:end)); %%m
ztop = outputs{33};
ztop = str2num(ztop(find(ztop=='=')+1:end)); %%m
interval = outputs{17};min_or_sec = interval(18);
outputrate = str2num(interval(find(interval=='=')+1:end));
if strcmp('m',min_or_sec)
  factor = 60;
else
  factor = 1;
end
outputrateseconds = outputrate*factor;

disp('Writing Parameters')
%%Output Parameters File
parameters = [dx;dy;ztop;outputrateseconds;dimZ;dimX;dimY]';
dlmwrite([simdataloc,'Parameters.txt'],parameters);
%%Output zcoord File
dlmwrite([simdataloc,'Zcoord.txt'],z');
%%Output Sample times in seconds
dlmwrite([simdataloc,'SampleTimes.txt'],(timestamps*outputrateseconds)');
%%output Terrain Height in meters (the transpose is there because
dlmwrite([simdataloc,'THeight.txt'],height,'delimiter',' ','precision','%.6f','newline','pc');

%We will have two for loops. One around the z-coordinate and
%another around the timestamp
disp('Consolidating Text Files')

disp('Consolidating U velocity')
for t = 1:length(timestamps)
  imtimestr = num2str(timestamps(t));
  outtimestr = num2str(timestamps(t)*outputrate/factor);
  disp(['U_Time = ',outtimestr,' ','seconds'])
  Uoutname = [simdataloc,'U',outtimestr,'.txt'];
  for zdx = 0:length(z)-1
    
    zstr = num2str(zdx);
    zcoord = z(zdx+1);
    
    %%%Import Temporary Files
    Uname = [rawdataloc,'U',zstr,'_',imtimestr,'.txt'];
    if ~exist(Uname)
      disp([Uname,' does not exist'])
      return
    end
    utemp = dlmread(Uname);
          
    %%%Output Arrays to Text Files
    if zdx == 0
      dlmwrite(Uoutname,utemp(end:-1:1,:),'delimiter',' ','precision','%.6f','newline','pc');
    else
      dlmwrite(Uoutname,utemp(end:-1:1,:),'delimiter',' ','precision','%.6f','newline','pc','-append');
    end
  end
end



disp('Consolidating V velocity')
for t = 1:length(timestamps)
  imtimestr = num2str(timestamps(t));
  outtimestr = num2str(timestamps(t)*outputrate/factor);
  disp(['V_Time = ',outtimestr,' ','seconds'])
  Voutname = [simdataloc,'V',outtimestr,'.txt'];
  for zdx = 0:length(z)-1
    
    zstr = num2str(zdx);
    zcoord = z(zdx+1);
    
    %%%Import Temporary Files
    Vname = [rawdataloc,'V',zstr,'_',imtimestr,'.txt'];
    if ~exist(Vname)
      disp([Vname,' does not exist'])
      return
    end
    
    vtemp = dlmread(Vname);
          
    %%%Output Arrays to Text Files
    if zdx == 0
      dlmwrite(Voutname,vtemp(end:-1:1,:),'delimiter',' ','precision','%.6f','newline','pc');
    else
      dlmwrite(Voutname,vtemp(end:-1:1,:),'delimiter',' ','precision','%.6f','newline','pc','-append');
    end
  end
end


disp('Consolidating W velocity')
for t = 1:length(timestamps)
  imtimestr = num2str(timestamps(t));
  outtimestr = num2str(timestamps(t)*outputrate/factor);
  disp(['W_Time = ',outtimestr,' ','seconds'])
  Woutname = [simdataloc,'W',outtimestr,'.txt'];
  for zdx = 0:length(z)-1
    
    zstr = num2str(zdx);
    zcoord = z(zdx+1);
    
    %%%Import Temporary Files
    Wname = [rawdataloc,'W',zstr,'_',imtimestr,'.txt'];
    if ~exist(Wname)
      disp([Wname,' does not exist'])
      return
    end
    
    wtemp = dlmread(Wname);
          
    %%%Output Arrays to Text Files
    if zdx == 0
      dlmwrite(Woutname,wtemp(end:-1:1,:),'delimiter',' ','precision','%.6f','newline','pc');
    else
      dlmwrite(Woutname,wtemp(end:-1:1,:),'delimiter',' ','precision','%.6f','newline','pc','-append');
    end
  end
end

disp('Finished Consolidating')