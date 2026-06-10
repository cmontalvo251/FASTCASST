purge
global NSTATES DELTA NOISE

addpath 'plotting';

NSTATES = 15;
STATES = 0;
NOISE = 0;
DELTA = 0;
CONTROLS = 0;
FORCES = 0;
BLEND = 0;
MOVIE = 0;
WINDS = 0;
RUNCODE = 1;

if RUNCODE
  system('rm Run.exe');
  system('rm source/Out_Files/*.OUT');
  system('~/Dropbox/BlackBox/compilec source/MMACSV10.cpp -O1 -w');
  system('./Run.exe source/Meta.ifiles');
end

%%Flight_movie prep
data = dlmread('source/Out_Files/Meta.OUT');
xout = data(:,2:end);
tout = data(:,1);
[r,c] = size(xout);
num_ac = round(c/NSTATES);
ts = 0.1;
timestep = tout(2)-tout(1);
wingspan = 2.04*ones(num_ac,1);
Az = 64;
El = 24;
xout = xout';
if MOVIE
  flight_movie
end

%%Plot States
if STATES
plotstates(3)
end
% %%Plot Controls
if CONTROLS
plotcontrols
end
%Plot Forces
if FORCES
plotforces
end
%%Plot Winds
if WINDS
plotwinds
end
%%Plot Blend Functions
if BLEND
plotblend
end







