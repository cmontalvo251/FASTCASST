%%This is the front end of uvwout.m, run this script
%before you run uvwout.m You only need to run it once
%close all
%clear
%clc

if ~exist('U0','var')

%%%%%%%%GLOBALS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global markX markY markZ markT parameters zcoord tcoord
global U0 Udt V0 Vdt W0 Wdt xcoord ycoord tlength tstr
global bounds boundflag tcoord zcoord terrain
global mark1 mark2

%%%%%%%%%%%Define Data Location%%%%%%%%%%%%%%

%dataloc = '~/Georgia_Tech/Grad_Research/WRF_Wind_Data/Flat_Terrain_Costello/';
%dataloc = '~/Georgia_Tech/Grad_Research/WindMapper/Wind_Modeling/Data/Flat_100x10/';
%dataloc = '~/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/';
%dataloc = '~/Work/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/';
%dataloc = 'C:\Users\carlos\Work\WRF_Wind_Data\Data25dx_HF_0\';
%dataloc = '~/Documents/WRF_Wind_Data/Data25dx_HF_0/';
%'c:/Root/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/';

disp(dataloc)

%%%%%%%%%%%Initialize Variables%%%%%%%%%%%%%

markX = 1;markY = 1;markZ = 1;markT = 1;
bounds = 0;boundflag = 1;
mark1 = [1 2];
mark2 = [1 2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%Import Extra Parameters File%%%%%%%%%

parameters = dlmread([dataloc,'Parameters.txt']);
zcoord = dlmread([dataloc,'Zcoord.txt']);
tcoord = dlmread([dataloc,'SampleTimes.txt']);
tlength = length(tcoord);
tstr = {};
for ii = 1:tlength
  tstr = [tstr{:} {num2str(tcoord(ii))}];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%Unwrap Parameters%%%%%%%%%%%%%%%

if length(parameters) == 7
  dimX = parameters(6);
  dimY = parameters(7);
  dimZ = parameters(5);
else
  dimZ = parameters(5);
  dimX = dimZ;
  dimY = dimZ;
end

dx = parameters(1);
dy = parameters(2);
ztop = parameters(3);
disp(['Grid Size = ',num2str(dx),'m'])
disp(['Maximum Height = ',num2str(ztop),'m'])
maxDim = dimX;

try
    disp('Terrain File Found')
    terrain = dlmread([dataloc,'THeight.txt']);
catch me
    disp('No Terrain File Found')
    terrain = zeros(maxDim,maxDim);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%Create xcoord and ycoord%%%%%%%
xcoord = -dx*(dimX-1)/2:dx:dx*(dimX-1)/2;
ycoord = -dy*(dimY-1)/2:dy:dy*(dimY-1)/2;

%%%%%%%Import Initial UVW matrices%%%%%%%%%
dtstr = num2str(tcoord(2));
U0name = [dataloc,'U0.txt'];
Udtname = [dataloc,'U',dtstr,'.txt'];
V0name = [dataloc,'V0.txt'];
Vdtname = [dataloc,'V',dtstr,'.txt'];
W0name = [dataloc,'W0.txt'];
Wdtname = [dataloc,'W',dtstr,'.txt'];
disp('Importing Wind Files')
U0 = importwind(U0name,dimX,dimY,dimZ);
Udt = importwind(Udtname,dimX,dimY,dimZ);
V0 = importwind(V0name,dimX,dimY,dimZ);
Vdt = importwind(Vdtname,dimX,dimY,dimZ);
W0 = importwind(W0name,dimX,dimY,dimZ);
Wdt = importwind(Wdtname,dimX,dimY,dimZ);
%U0 = U0(end:-1:1,end:-1:1,:);
%V0 = V0(end:-1:1,end:-1:1,:);
%W0 = W0(end:-1:1,end:-1:1,:);
%Udt = Udt(end:-1:1,end:-1:1,:);
%Vdt = Vdt(end:-1:1,end:-1:1,:);
%Wdt = Wdt(end:-1:1,end:-1:1,:);
disp('UVW Initialization Complete')

else
  disp('Skipped Loading WindField')
end


% Copyright - Carlos Montalvo 2015
% You may freely distribute this file but please keep my name in here
% as the original owner
