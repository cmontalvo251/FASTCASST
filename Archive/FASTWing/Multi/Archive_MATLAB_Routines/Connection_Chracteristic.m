%%%%MONTE CARLO STUFF%%%%
purge

format long g
files = {'AC.txt'};

%%%Create Configuration File
config_data = {'20 20 !Grid_Size_of_Configuration'};
for ii = 1:20
  config_data = [config_data;num2str(zeros(1,10))];
end
config_data = [config_data;'10000 10000 !KLtangent_KLnormal';'40 40 !CLtangent_CLnormal';'1000 !Maximum_Magnet_Force';'372.1041 2584.05625 2584.05625 !KR';'1.4884 10.33623 10.33623 !CR'];
writematrix = zeros(20,20);
num_ac = 2;
W2W = 1;
configuration = ones(num_ac,1);
if W2W
  writematrix(1,1:num_ac) = configuration';
else
  writematrix(1:num_ac,1) = configuration;
end
for idx = 1:20
  config_data{idx+1} = num2str(writematrix(idx,:));
end
writedata('source/Input_Files/Configuration.CONN',config_data);

%%%%%VARY A SINGLE PARAMETER%%%%%%%
N = 1000;
param = linspace(0.0000000001,100,N);
param = param(end:-1:1);
KR1 = 372.1041;
KR2 = 2584.05625;
KR3 = 2584.05625;
CR1 = 1.4884;
CR2 = 10.33623;
CR3 = 10.33623;

%%%%%CREATE VECTOR ARRAYS FOR ALL MODES%%%%%
N = length(param);
all_modes = zeros(24,N);
flapping = zeros(2,N);
pitch = zeros(2,N);
leadlag = zeros(2,N);

for ii = 1:length(param)
  disp([num2str(ii),' out of ',num2str(length(param))])
  
  %%%Change parameter in configuration file
  config_data{end-1} = [num2str(KR1),' ',num2str(KR2),' ',num2str(KR3),' !KR'];
  config_data{end} = [num2str(CR1),' ',num2str(CR2),' ',num2str(param(ii)),' !CR'];
  writedata('source/Input_Files/Configuration.CONN',config_data);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %%%%%RUN CODE
  system('Run.exe source/MultiMeta.ifiles 1> runfile');
  
  %%%%%%%%SEARCH FOR MODES
  data = dlmread(['source/Out_Files/Linearized/',files{1}]);
  [r,c] = size(data);
  Alinear = data(1:c,1:c);
  [r,c] = size(Alinear);
  NSTATE = 15;
  num_ac = round(r/NSTATE);
  Blinear = data(NSTATE*num_ac+1:end,1:num_ac*3);
  Alinear = Alinear(1:(num_ac*NSTATE),1:(num_ac*NSTATE));
  delrows = [];
  angles = [];
  for jj = 1:num_ac
    s = (jj-1)*NSTATE;
    s2 = (jj-1)*12;
    delrows = [delrows,(s+13):(s+12+3)];
    angles = [angles,(s2+4):(s2+6)];
  end
  Alinear(delrows,:) =  [];
  Alinear(:,delrows) =  [];
  Blinear(delrows,:) = [];
  Alinear;
  Blinear;
  
  [vectors,acmodes] = eig(Alinear);
  acmodes = eig(Alinear);
  all_modes(:,ii) = acmodes;
  %%%Search for Modes
  pctr = 1;
  llctr = 1;
  fctr = 1;
  for kk = 1:length(acmodes)
    eigenvector = vectors(:,kk);
    %%%Normalize all values
    for ll = 1:length(eigenvector)
        eigenvector(ll) = norm(eigenvector(ll));
    end
    %%%%Search for the most dominant modes
    [sorted,idx] = sort(eigenvector);
    %%%Grab the last 2 indeices
    dominant = idx(end:-1:end-1);
    %%%Pitch mode is q1,q2
    dominant = sort(dominant);
    if dominant == [11 23]'
        %%%This can also be the short period pitch mode so we need to check
        %%%the signs (q1 = -12)
        eigenvector = real(vectors(:,kk));
        if (sign(eigenvector(11)) ~= sign(eigenvector(23)))
            disp('Flexible Pitch Mode')
            disp(acmodes(kk))
            pitch(pctr,ii) = acmodes(kk);
            pctr = pctr + 1;
            if pctr == 3
                pctr = 1;
            end
        end
    end
    %%%Lead Lag Mode is v1,v2,r1,r2
    if dominant == [12 24]'
      eigenvector = real(vectors(:,kk));
      %%%This could be a constraint mode so we need to make sure (r1=-r2)
      if (sign(eigenvector(12)) ~= sign(eigenvector(24)))
        disp('Lead Lag Mode')
        disp(acmodes(kk))
        leadlag(llctr,ii) = acmodes(kk);
        llctr = llctr + 1;
        if llctr == 3
          llctr = 1;
        end
      end
    end
    %%%Flapping is v1,v2,p1,p2
    if dominant == [10 22]'
      %%%This could be a constraint mode or the roll mode so we
      %need to make sure (p1=-p2)
      eigenvector = real(vectors(:,kk));
      if (sign(eigenvector(10)) ~= sign(eigenvector(22)))
        disp('Flapping Mode')
        disp(acmodes(kk))
        flapping(fctr,ii) = acmodes(kk);
        fctr = fctr + 1;
        if fctr == 3
           fctr = 1;
        end
      end
    end
    
    
  end %%kk = 1:length(acmodes)
  %%Check and make sure that all modes are accounted for
  if abs(flapping(1,ii)) == 0 && ii > 1
    %%%This means that the dominant mode has switched so we must
    %compare the previous mode with all the eigen vectors
    acmodes_backup = acmodes;
    %%Look for the first mode
    [val,loc] = min(abs(real(flapping(1,ii-1))-real(acmodes)));
    flapping(1,ii) = acmodes(loc);
    %%%Look for the second mode
    acmodes(loc) = [];
    [val,loc] = min(abs(real(flapping(2,ii-1))-real(acmodes)));
    flapping(2,ii) = acmodes(loc);
    disp('Flapping Mode')
    disp(flapping(:,ii))
    acmodes = acmodes_backup;
  end
  if ii > 1
    if (abs(leadlag(1,ii)) == 0 || abs(leadlag(2,ii)) == 0)
    %%%This means that the dominant mode has switched so we must
    %compare the previous mode with all the eigen vectors
    acmodes_backup = acmodes;
    %%%First throw out roll mode
    [val,loc] = min(abs(-7.95-real(acmodes)));
    acmodes(loc) = [];
    %%%Throw out the last 4 rigid modes 
    acmodes(end:-1:end-3) = [];
    %%%Throw out the short period
    [val,loc] = min(abs(real(-6.02935707033183 +      9.23207802854664i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(-6.02935707033183 +      9.23207802854664i)-real(acmodes)));
    acmodes(loc) = [];
    %%%Throw out the long period
    [val,loc] = min(abs(real(  -0.0308120546077461 +     0.599308108797243i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(  -0.0308120546077461 +     0.599308108797243i)-real(acmodes)));
    acmodes(loc) = [];
    %%%Throw out spiral mode
    [val,loc] = min(abs(real(0.0736485303365114)-real(acmodes)));     
    acmodes(loc) = [];
    %%%Dutch roll mode
    [val,loc] = min(abs(real(     -0.2393  +      0.857i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(      -0.2393 +       0.857i)-real(acmodes)));
    acmodes(loc) = [];
    %%Throw out flapping and pitch mode
    [val,loc] = min(abs(real(     -11.999 +      38.888i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(      -11.999 +       38.888i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(     -22.49 +      98.5i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(      -22.49 +     98.5i)-real(acmodes)));
    acmodes(loc) = [];
    %%%Throw out the constraint modes
    [val,loc] = min(abs(real(     -7.91035 +      315.59i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(      -7.91035 +       315.59i)-real(acmodes)));
    acmodes(loc) = [];
    %%%Throw out the constraint modes
    [val,loc] = min(abs(real(     -7.269 +      59.245i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(      -7.269 +       59.245i)-real(acmodes)));
    acmodes(loc) = [];
    %%%Throw out the constraint modes
    [val,loc] = min(abs(real(     -0.2392 +      0.85712i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(      -0.2392 +       0.85712i)-real(acmodes)));
    acmodes(loc) = [];
    %%Look for the first mode
    [val,loc] = min(abs(real(leadlag(1,ii-1))-real(acmodes)));
    leadlag(1,ii) = acmodes(loc);
    %%%Look for the second mode
    acmodes(loc) = [];
    [val,loc] = min(abs(real(leadlag(2,ii-1))-real(acmodes)));
    leadlag(2,ii) = acmodes(loc);
    disp('Lead Lag Mode')
    disp(leadlag(:,ii))
    acmodes = acmodes_backup;
    end
  end
  if ii > 0
    if (abs(pitch(1,ii)) == 0 || abs(pitch(2,ii)) == 0)
    %%%This means that the dominant mode has switched so we must
    %compare the previous mode with all the eigen vectors
    acmodes_backup = acmodes;
    %%%First throw out roll mode
    %[val,loc] = min(abs(-16.34-real(acmodes)));
    %acmodes(loc) = [];
    %%%Throw out the last 4 rigid modes 
    acmodes(end:-1:end-3) = [];
    %%%Throw out the short period
    [val,loc] = min(abs(real(-6.02935707033183 +      9.23207802854664i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(-6.02935707033183 +      9.23207802854664i)-real(acmodes)));
    acmodes(loc) = [];
    %%%Throw out the long period
    [val,loc] = min(abs(real(  -0.0308120546077461 +     0.599308108797243i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(  -0.0308120546077461 +     0.599308108797243i)-real(acmodes)));
    acmodes(loc) = [];
    %%%Throw out spiral mode
    [val,loc] = min(abs(real(0.0736485303365114)-real(acmodes)));     
    acmodes(loc) = [];
    %%Throw out flapping and lead lag
    [val,loc] = min(abs(real(     -11.999 +      38.888i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(      -11.999 +       38.888i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(     -12.46 +      72.6i)-real(acmodes)));
    acmodes(loc) = [];
    [val,loc] = min(abs(real(      -12.46 +     72.6i)-real(acmodes)));
    acmodes(loc) = [];
    %%Look for the first mode
    [val,loc] = min(abs(real(pitch(1,ii-1))-real(acmodes)));
    pitch(1,ii) = acmodes(loc);
    %%%Look for the second mode
    acmodes(loc) = [];
    [val,loc] = min(abs(real(pitch(2,ii-1))-real(acmodes)));
    pitch(2,ii) = acmodes(loc);
    disp('Pitch Mode')
    disp(pitch(:,ii))
    acmodes = acmodes_backup;
    end
  end

end
pitch = pitch';
flapping = flapping';
leadlag = leadlag';
save Modes.mat

plottool(1,'Pitch',12,'Real','Imaginary');
plot(pitch,'k-x','LineWidth',2)

plottool(1,'Flapping',12,'Real','Imaginary');
plot(flapping,'k-x','LineWidth',2)

plottool(1,'Lead Lag',12,'Real','Imaginary');
plot(leadlag,'k-x','LineWidth',2)


plottool(1,'All',12,'Real','Imaginary');

p1 = plot(real(pitch(:,1)),imag(pitch(:,1)),'k-s','LineWidth',2);
plot(real(pitch(:,2)),imag(pitch(:,2)),'k-s','LineWidth',2);

p2 = plot(real(flapping(:,1)),imag(flapping(:,1)),'k-x','LineWidth',2);
plot(real(flapping(:,2)),imag(flapping(:,2)),'k-x','LineWidth',2);

p3 = plot(real(leadlag(:,1)),imag(leadlag(:,1)),'k-d','LineWidth',2);
plot(real(leadlag(:,2)),imag(leadlag(:,2)),'k-d','LineWidth',2);
legend([p1(1) p2(1) p3(1)],'Pitch Mode','Flapping Mode','LeadLag Mode')