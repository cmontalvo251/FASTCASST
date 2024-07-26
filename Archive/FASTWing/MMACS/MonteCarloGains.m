purge

system('rm Run.exe');
system('~/Dropbox/BlackBox/compilec source/MMACSV10.cpp -O1 -w');

depenalty = linspace(0.1,100,10);
thetapenalty = linspace(0.1,100,10);

%match  = dlmread('/home/carlos/Work/Grad_Research/Meta-Aircraft/Archive_MMACS/MMACS/MMACSNOMINALAPRIL/Out_Files/Meta.OUT');
[h1,ax1] = plottool(1,'theta',12,'Time(sec)','theta(rad)');
%[h2,ax2] = plottool(1,'phi',12,'Time(sec)','phi(rad)');
%[h3,ax3] = plottool(1,'y',12,'Time(sec)','y(m)');
for ii = 1:length(depenalty)
  for jj = 1:length(thetapenalty)
    %%dlmwrite
    dlmwrite('Gain.txt',[depenalty(ii) thetapenalty(jj)],'delimiter',' ');
    [depenalty(ii) thetapenalty(jj)]
    % cla(ax1)
    % cla(ax2)
    %%runcode
    system('./Run.exe source/Meta.ifiles &> file');
    %%save results
    data = dlmread('source/Out_Files/Meta.OUT');
    tout = data(:,1);
    val = data(:,1*15+1+5);
    plot(ax1,tout,val)
    % val2 = data(:,0*15+1+4);
    % plot(ax2,tout,val2)
    % val3 = data(:,0*15+1+2);
    % plot(ax3,tout,val3)
    drawnow
    %pause
  end
end

  
