purge

tic

system('rm Run.exe');
system('rm Tracking_Performance.txt');
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

timefile = {'0	!Tinitial';
'100	!Tfinal_(0.559)';
'0.001	!Timestep(no_contact)';
'0.001	!Timestep(with_contact)';
'10	!Skip(Skip*timestep_is_the_output_rate)';
'0.0	!X(m)';
'0	!Y';
'-200	!Z';
'0.0	!phi';
'0.0	!theta(0.09057)';
'0	!psi';
'20	!u(m/s)_(parent)';
'0	!v';
'0	!w(1.5)';
'0.0	!p';
'0	!q';
'0	!r';
'0	!X(second_ac)';
'6.12	!Y';
'-200.0 	!Z';
'0.0	!phi(0.5)';
'0.0	!theta(0.1957)';
'0.0	!psi(-0.5)';
'20	!u(m/s)(17)';
'0.0	!v(2.0)';
'0.0	!w(-1.5)';
'0.0	!p(0.3)';
'0.0     !q(-1)';
'0.0	!r'};

%%Nominal
%windlevel = 0;
%turblevel = 0;
writedata('source/Input_Files/Meta.TIME',timefile);
writedata('source/Input_Files/Meta.WIND',windfile);
%system('./Run.exe source/Meta.ifiles &> runfile');
%%Read Data and get nominal tracking performance
%data = dlmread('source/Out_Files/Meta.OUT');
%tout = data(:,1);
%xout = data(:,2:end);
%%%Compute the tracking coordinates
%psi0 = xout(1,6);
%Vtrim = 20;
%xtrack = Vtrim*tout*cos(psi0);
%ytrack = Vtrim*tout*sin(psi0);
%%%Compute perterbations from that point
%xout(:,1) = xout(:,1) - xtrack;
%xout(:,2) = xout(:,2) - ytrack;
%%%Clip to after trim
%l = find(tout > 40,1);
%xout = xout(l:end,:);
%%%Process data
%meanX = mean(abs(xout))
%minX = min(xout)
%maxX = max(abs(xout))
%stdX = std(xout)
%%%Append data to file
%dlmwrite('Tracking_Performance.txt',[windlevel,turblevel],'delimiter',' ','-append');
%dlmwrite('Tracking_Performance.txt',[meanX;minX;maxX;stdX],'delimiter','
%','-append');
%return

%%%%loop through different windlevels and get data
windlevels = 0:0.1:3;
heading = linspace(0,2*pi,20);
%heading = linspace(0,0,1);
for ii = 1:length(windlevels)
    %%%%Set wind level
    windlevel = windlevels(ii);
    turblevel = windlevel;
    strturb = [num2str(turblevel),' '];
    strwrf = [num2str(windlevel),' '];
    windfile{5} = [strturb strturb strturb ,'   !Turbulence_Scale'];
    windfile{7} = [strwrf strwrf strwrf ,'   !WRF_Scale']
    %%%Write data
    writedata('source/Input_Files/Meta.WIND',windfile);
    for jj = 1:length(heading)
        %%%Set heading
        psi0 = heading(jj)
        timefile{11} = [num2str(psi0),'   !psi'];
        %%%Writedata
        writedata('source/Input_Files/Meta.TIME',timefile);
        %%%Run code
        system('./Run.exe source/Meta.ifiles &> runfile');
        %%Read Data and get nominal tracking performance
        data = dlmread('source/Out_Files/Meta.OUT');
        tout = data(:,1);
        xout = data(:,2:end);
        %%%Compute the tracking coordinates
        psi0 = xout(1,6);
        Vtrim = 20;
        xtrack = Vtrim*tout*cos(psi0);
        ytrack = Vtrim*tout*sin(psi0);
        %%%Compute perterbations from that point
        xout(:,1) = xout(:,1) - xtrack;
        xout(:,2) = xout(:,2) - ytrack;
        xout(:,3) = xout(:,3) + 200;
        %%theta,psi
        xout(:,5) = xout(:,5) - 0.13;
        xout(:,6) = xout(:,6) - psi0;
        %%%Clip to after trim
        l = find(tout > 40,1);
        xout = xout(l:end,:);
        %%%Process data
        meanX = mean(abs(xout))
        minX = min(abs(xout))
        maxX = max(abs(xout))
        stdX = std(xout)
        %%%Append data to file
        dlmwrite('Tracking_Performance.txt',[psi0,windlevel,turblevel],'delimiter',' ','-append');
        dlmwrite('Tracking_Performance.txt',[meanX;minX;maxX;stdX],'delimiter',' ','-append');
    end
end


toc