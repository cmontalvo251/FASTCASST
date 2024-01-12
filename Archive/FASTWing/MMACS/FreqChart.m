purge

addpath 'plotting';

system('rm Run.exe');
system('~/Dropbox/BlackBox/compilec source/MMACSV10.cpp -w -O1');

%%Reset Windfile
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
'40	!Tfinal_(0.559)';
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

writedata('source/Input_Files/Meta.TIME',timefile);
writedata('source/Input_Files/Meta.WIND',windfile);


system('./Run.exe source/Meta.ifiles');
disp('Importing States')
Body = dlmread('source/Out_Files/Meta.OUT');
toutNominal = Body(:,1);
Gftimestep = toutNominal(2);
xoutNominal2 = Body(:,17:end);
xoutNominal = Body(:,2:16);

%plotstates
%break

%frequencies = [0 0.05 0.11];
%frequencies = 0.1;
%frequencies = [0:0.005:0.1];
frequencies = linspace(0,0.1,40);
%speeds = [0:0.01:1.5];
speeds = 0.8;
%frequencies = 0;
%xCost = zeros(length(frequencies),11);
xCost = zeros(length(speeds),length(frequencies),11);

for ii = 1:length(speeds)
  for jj = 1:length(frequencies)
    %%%Edit Windfile
    if frequencies(jj) == 0
      s = 0;
    else
      s = speeds(ii);
    end
    windfile{2} = ['0.0  ',num2str(s),'  0.0 !Contant_Scale'];
    windfile{3} = ['0.0  ',num2str(frequencies(jj)),'  0.0 !Frequency_Scale'];
    writedata('source/Input_Files/Meta.WIND',windfile);
    system('cat source/Input_Files/Meta.WIND');
    system('./Run.exe source/Meta.ifiles &> Outfile');
    disp('Importing States')
    Body = dlmread('source/Out_Files/Meta.OUT');
    tout = Body(:,1);
    timestep = tout(2);
    xout2 = Body(:,17:end);
    xout = Body(:,2:16);
    %%%Measure Cost from Nominal
    %%Use this for Wgust
    %xCost(ii,jj,:) = max(xout2(find(tout>5,1):end,2:12)-xoutNominal2(find(tout>5,1):end,2:12));
    %Use this for Vgust
    xCost(ii,jj,:) = max(xout2(find(tout>20,1):end,2:12)-xoutNominal2(find(tout>20,1):end,2:12));
    %Use this for parent
    %xCost(ii,jj,:) = max(xout(find(tout>10,1):end,2:12)-xoutNominal(find(tout>10,1):end,2:12));
  end
end
%load N1.mat
%%Last row and first row are dummy rows
%%Must be positive
xCost = abs(xCost);
[r,c,d] = size(xCost);
dim1 = 'm';
col = {'k','k--','k-s'};
ylabels = {['y(',dim1,')'],['z(',dim1,')'],'\phi(deg)','\theta(deg)','\psi(deg)',['u(',dim1,'/s)'],['v(',dim1,'/s)'],['w(',dim1,'/s)'],'p(rad/s)','q(rad/s)','r(rad/s)'};
for ii = 1:d
  plottool(1,'Cost',12,'Frequencies(rad/m)',ylabels{ii});
  %plottool(1,'Cost',12,'WindSpeed(m/s)',ylabels{ii});
  factor = 1;
  if ii >= 3 && ii <= 5
    factor = 180/pi;
  end
  legends = {};
  % for jj = 1:length(frequencies)
  %   plot(speeds,factor.*xCost(:,jj,ii),col{jj},'LineWidth',2);
  %   legends = [legends,[num2str(frequencies(jj)*20),' rad/s']];
  % end
  legend(legends)
  % plot(frequencies(2:end).*20,factor.*xCost1(:,2:end,ii),'LineWidth',2)
  plot(frequencies(1:end-1),factor.*xCost(:,2:end,ii),'k-','LineWidth',2)
  % legend('N=2','N=1')
  %legend('Child','Parent')
end
plottool(1,'Cost',12,'Frequencies(rad/m)','|x(f_x)-x(0)|')
a = length(frequencies);
t = zeros(a,11);
for ii = 1:a
  for jj = 1:11
    t(ii,jj) = xCost(1,ii,jj);
  end
end
tnorm = zeros(1,a);
for ii = 1:a
  tnorm(ii) = norm(t(ii,:));
end
plot(frequencies,tnorm,'k-','LineWidth',2)
%xlim([0 frequencies(end)])
% xCost1 = xCost;
% clear xCost;
% save N1




