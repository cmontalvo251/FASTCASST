purge

global NSTATES num_ac mass I_b I_b_inv rcg2ac phic thetac

addpath 'plotting';

NSTATES = 15;
DELTA = 0;
CONTROLS = 1;
FORCES = 0;
BLEND = 0;
MOVIE = 0;
WINDS = 0;
STATES = 1;
RUNCODE = 1;
AC = 2; 
W2W = 1;

config_data = {'20 20 !Grid_Size_of_Configuration'};
for ii = 1:20
    config_data = [config_data;num2str(zeros(1,10))];
end
config_data = [config_data;'10000 10000 !KLtangent_KLnormal';'40 40 !CLtangent_CLnormal';'1000 !Maximum_Magnet_Force';'372.1041 2584.05625 2584.05625 !KR';'1.4884 10.33623 10.33623 !CR'];
%config_data = [config_data;'10000 10000 !KLtangent_KLnormal';'40 40 !CLtangent_CLnormal';'1000 !Maximum_Magnet_Force';'0.1 0.1 0.1 !KR';'1.4884 10.33623 10.33623 !CR'];

for num_ac_ctr = AC
    
    num_ac = num_ac_ctr

    %%Numerically obtain it
    writematrix = zeros(20,20);
    configuration = ones(num_ac,1);
    if W2W
      writematrix(1,1:num_ac) = configuration';
    else
      writematrix(1:num_ac,1) = configuration;
    end
    for idx = 1:20
      config_data{idx+1} = num2str(writematrix(idx,:));
    end
    writedata('Input_Files/Configuration.CONN',config_data);

    if RUNCODE
      system('make');
      system('./Run.exe Input_Files/MultiMeta.ifiles');                    
      num_col = (num_ac-1)*W2W+1;
      newname = ['Meta',num2str(num_col),'_',num2str(num_ac)];
      system(['cp Out_Files/Meta.OUT Out_Files/',newname]);
      newname = ['Meta_Control',num2str(num_col),'_',num2str(num_ac)];
      system(['cp Out_Files/Meta_Control.OUT Out_Files/',newname]);
    end

    %%Flight_movie prep
    filename = 'Out_Files/Meta.OUT';
    controlname = 'Out_Files/Meta_Control.OUT';

    if MOVIE || STATES
      data = dlmread('Out_Files/Meta.OUT');
      xout = data(:,2:end);
      tout = data(:,1);
      [r,c] = size(xout);
      num_ac = round(c/NSTATES);
      SetupConfiguration
      ts = 0.01;
      timestep = tout(2)-tout(1);
      wingspan = 2.04*ones(num_ac,1);
      Az = 64;
      El = 24;
      xout = xout';
      if MOVIE
        flight_movie
      end
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

    %playsound
    %pause
end
