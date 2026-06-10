purge

global NSTATES num_ac mass I_b I_b_inv rcg2ac phic thetac

addpath 'plotting';

NSTATES = 15;
STATES = 1;
DELTA = 0;
CONTROLS = 0;
FORCES = 0;
BLEND = 0;
MOVIE = 0;
WINDS = 0;
RUNCODE = 1;

%%%COMPUTE SPEED FIRST
%%%Compute Speed of given configuration
%%First comptute maximum pitch rate
%disp('Compute Pitch Rate Dynamics')
%phic = 0;
%thetac = 30*pi/180;
%[wnq,kpz,d,d,qmax,d] = ComputeSpeed();
%%Then compute maximum roll rate
%disp('Compute Roll Rate Dynamics')
%thetac = 0;
%phic = 30*pi/180;
%[d,d,wnp,kppsi,d,pmax] = ComputeSpeed();

%%%Write all this data to a file
%fid = fopen('source/Input_Files/Meta.FREQ','w');
%data = {[num2str(wnq),' !wnq'];
% 	[num2str(kpz),' !kpz'];
% 	[num2str(wnp),' !wnp'];
% 	[num2str(kppsi),' !kppsi'];
% 	[num2str(qmax),' !qmax'];
% 	[num2str(pmax),' !pmax']};
% for ii = 1:length(data)
%   fprintf(fid,'%s \n',data{ii});
% end

if RUNCODE
  system('make');
  system('./Run.exe source/MultiMeta.ifiles');                    
end

%%Flight_movie prep
filename = 'source/Out_Files/Meta.OUT';
controlname = 'source/Out_Files/Meta_Control.OUT';
data = dlmread(filename);
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
  plotstates([1:12],filename)
end
% %%Plot Controls
if CONTROLS
  plotcontrols(controlname)
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