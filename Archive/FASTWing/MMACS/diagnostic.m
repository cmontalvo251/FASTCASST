purge

system('rm Run.exe');
system('rm source/Out_Files/*.OUT');
system('~/Dropbox/BlackBox/compilec source/MMACSV10.cpp -O1 -w');

windfile = {'1		!Constant_Winds';
' 0.0  0.0  0.0	!Constant_Scale';
'0.0 0.0 0.0	!Frequency_Scale';
'0		!Turbulence';
'1 1 1		!Turbulence_Scale';
'1		!WRF';
'0 0 0		!WRF_Scale';
'0		!Override_Path';
'/localhome/WRF_Wind_Data/Data25dx_HF_0/'}

%%Nominal
writedata('source/Input_Files/Meta.WIND',windfile);
system('./Run.exe source/Meta.ifiles &> runfile');
a = dlmread('Connection.txt');
if (~a)
  disp('Connection Failed')
  return
else
  disp('Connection Achieved')
  disp(a)
end

%%Constant winds
for ii = 1:3
  for jj = 1:2
    if jj == 1
      val = [' ',num2str(0.9)];
    else
      val = num2str(-0.9);
    end
    windfile{2}((1+(ii-1)*5):(4+(ii-1)*5)) = val
    writedata('source/Input_Files/Meta.WIND',windfile);
    system('./Run.exe source/Meta.ifiles &> runfile');
    a = dlmread('Connection.txt');
    if (~a)
      disp('Connection Failed')
      return
    else
      disp('Connection Achieved')
      disp(a)
    end
  end
  windfile{2}((1+(ii-1)*5):(4+(ii-1)*5)) = ' 0.0';
end
%%Turbulence
windfile{4}(1) = '1'
writedata('source/Input_Files/Meta.WIND',windfile);
system('./Run.exe source/Meta.ifiles &> runfile');
a = dlmread('Connection.txt');
if (~a)
  disp('Connection Failed')
  return
else
  disp('Connection Achieved')
  disp(a)
end
windfile{4}(1) = '0';

%%Wrf
for ii = 1:3
  windfile{7}(1+(ii-1)*2) = '1'
  writedata('source/Input_Files/Meta.WIND',windfile);
  system('./Run.exe source/Meta.ifiles &> runfile');
  a = dlmread('Connection.txt');
  windfile{7}(1+(ii-1)*2) = '0';
  if (~a)
    disp('Connection Failed')
    return
  else
    disp('Connection Achieved')
    disp(a)
  end
end

%%Wrf u,v
windfile{7}(1) = '1';
windfile{7}(3) = '1'
writedata('source/Input_Files/Meta.WIND',windfile);
system('./Run.exe source/Meta.ifiles &> runfile');
a = dlmread('Connection.txt');
if (~a)
   disp('Connection Failed')
   break
else
  disp('Connection Achieved')
  disp(a)
end
%%Wrf uvw
windfile{7}(5) = '1'
writedata('source/Input_Files/Meta.WIND',windfile);
system('./Run.exe source/Meta.ifiles &> runfile');
a = dlmread('Connection.txt');
if (~a)
   disp('Connection Failed')
   break
else
  disp('Connection Achieved')
  disp(a)
end



