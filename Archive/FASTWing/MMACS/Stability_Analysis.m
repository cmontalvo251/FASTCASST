purge

system('rm Run.exe');
system('~/Dropbox/BlackBox/compilec source/MMACSV10.cpp -w -O1');
system('./Run.exe source/Meta.ifiles');

%files = {'Single_Aircraft_Single_Computation_Point.txt','Single_Aircraft.txt'};
files = {'Dummy.txt'};

[h1,f1] = plottool(1,'Modes',12,'Real','Imag');
colors = {'b*','ro','gs','m+','cd'};
for ii = 1:length(files)
  data = dlmread(['source/Out_Files/Linearized/',files{ii}]);
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
  Alinear
  Blinear

  acmodes = eig(Alinear)
  [eigenvectors,singularvalues] = eig(Alinear);
  
  plotmodes = acmodes;
  real_comp = real(plotmodes);
  imag_comp = imag(plotmodes);
  
  plot(f1,real_comp,imag_comp,colors{ii},'MarkerSize',12);
end
