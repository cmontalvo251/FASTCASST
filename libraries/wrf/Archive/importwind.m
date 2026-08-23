function windout = importwind(name,dimX,dimY,dimZ)
%%this function will take in the name of a file and the 
%%max dimension of the WRF simulation and import the 
%%said text file into a 3dimensional array

%%Format of input file is this U#.txt
%# corresponds to the time and "U" corresponds to the wind (U,V,W)
%The text file is laid out in a concatenated matrix format:

%x - y data at z(i=0)
%x - y data at z(i=1)
%...
%x -y data at z(i=dimZ)

%Thus if dimX = 100, dimY = 2, dimZ = 20 
%the text file would be a 40 x 100 matrix


windout = zeros(dimY,dimX,dimZ);

importwindfile = fopen(name);
if importwindfile == -1
    disp(['sorry ',name,' does not exist'])
end
disp(['Importing Wind File: ',name])
for jj = 1:dimZ
  for ii = 1:dimY
    par = str2num(fgetl(importwindfile));
    windout(ii,:,jj) = par;
  end
end

fclose(importwindfile);

% Copyright - Carlos Montalvo 2015
% You may freely distribute this file but please keep my name in here
% as the original owner
