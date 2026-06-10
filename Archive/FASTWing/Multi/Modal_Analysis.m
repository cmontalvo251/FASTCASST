purge
global STLREAD xyzf xyzL xyzR xyzp

NoStates = 15;
RUNCODE = 1;
MOVIES = 1;
timestep = 0.001;
ts = timestep*4;
tend = 1;
files = {'AC.txt'};

if RUNCODE
  system('make');
  system('./Run.exe Input_Files/MultiMeta.ifiles');
end

if MOVIES
  %%Flight_movie prep
  data = dlmread('Out_Files/Meta.OUT');
  xout = data(:,2:end);
  tout = data(:,1);
  [r,c] = size(xout);
  num_ac = round(c/NoStates);
  wingspan = 2.04*ones(num_ac,1);
  Az = 64;
  El = 24;
  xout = xout';
  NSTATES = 12;
end

[h1,f1] = plottool(1,'Modes',12,'Real','Imag');
colors = {'kx','ro','gs','m+','cd'};
for idx = 1:length(files)
  data = dlmread(['Out_Files/Linearized/',files{idx}]);
  [r,c] = size(data);
  Alinear = data(1:c,1:c);
  [r,c] = size(Alinear);
  num_ac = round(r/NoStates);
  Blinear = data(NoStates*num_ac+1:end,1:num_ac*4);
  Alinear = Alinear(1:(num_ac*NoStates),1:(num_ac*NoStates));

  delrows = [];
  angles = [];
  for jj = 1:num_ac
    s = (jj-1)*NoStates;
    s2 = (jj-1)*12;
    delrows = [delrows,(s+13):(s+12+3)];
    angles = [angles,(s2+4):(s2+6)];
  end
  Alinear(delrows,:) =  [];
  Alinear(:,delrows) =  [];
  Blinear(delrows,:) = [];
  
  %%%Combine States to remove degrees of freedom
  T = zeros((12-3*(num_ac-1))*num_ac,12*num_ac);
  %%%Combine x,y,z
  for ii = 1:3
      cols = ii:12:(12*num_ac);
      for jj = cols
        T(ii,jj) = 1/num_ac;
      end
  end
  %%%Combine u,v,w
  for ii = 4:6
      cols = (7+ii-4):12:(12*num_ac);
      for jj = cols
        T(ii,jj) = 1/num_ac;
      end
  end
  %%%Now set everything else
  [r,c] = size(T);
  I3 = eye(3);
  for ii = 1:(2*num_ac)
      s = 7 + 3*(ii-1);
      rr = s:(s+2);
      s = 4 + 6*(ii-1);
      cc = s:(s+2);
      T(rr,cc) = I3;
  end
  %%%%Compute the Transform
  Btilde = T*Blinear;
  Atilde = T*Alinear*T'*inv(T*T');
  
  %%%%Test for controllability
  Wc = controllability(Alinear,Blinear);
  size(Alinear)
  r = rank(Wc)
  
  Wc = controllability(Atilde,Btilde);
  size(Atilde)
  r = rank(Wc)
  
  acmodes = eig(Alinear);
  l = length(acmodes);
  jj = 1;
  while jj <= l
      %fprintf('%d %f + %fi \n',jj,real(acmodes(jj)),imag(acmodes(jj)))
      fprintf('%f + %fi \n',real(acmodes(jj)),imag(acmodes(jj)))
      jj = jj + 1;
  end
  %acmodes(acmodes==0)=[];
  [eigenvectors,singularvalues] = eig(Alinear);
  
  plotmodes = acmodes;
  real_comp = real(plotmodes);
  imag_comp = imag(plotmodes);
  
  %plot(f1,real_comp,imag_comp,colors{idx},'MarkerSize',10);
  
  if MOVIES
    nn = 0;
    format short g
   	xnominal0 = dlmread(['Out_Files/Linearized/',files{idx}(1:end-4),'.nom']);
    while nn < 12*num_ac
      %%Check for Real or Imaginary
      nn = nn + 1;
      current_mode = acmodes(nn);
      if abs(current_mode) > 1e-5
        disp('Eigen Value = ')
        acmodes(nn)
        nn
        X = real(eigenvectors(:,nn));
        xinitial = 2*X;
        Y = imag(eigenvectors(:,nn));
        %sig = real(current_mode);
        sig = -2;
        if (abs(imag(current_mode))<1e-4)
            w = 0;
            tstart = 0;
        else
            w = 31.5;
            tstart = atan2(X(4),Y(4))/w;
        end
        toutL = (tstart):timestep:(tend+tstart);
        xoutL = zeros(length(toutL),12*num_ac);
    	for jj = 1:(12*num_ac)
          xoutL(:,jj) = 2.*exp(sig.*toutL').*(X(jj).*cos(w.*toutL')-Y(jj).*sin(w.*toutL'));
        end
      	toutL = 0:timestep:tend;
        xnominal = xnominal0(1:(NoStates*num_ac),1);
    	xnominal(delrows)=[];
        [r,c] = size(xoutL);
	
        scalef = max(max(abs(xoutL(:,angles))));
        scale = (20*pi/180)/scalef;
        if scale == Inf
            scale = 1;
        end
        %scale = 1;
        xout = scale.*xoutL + ones(r,1)*xnominal';
        tout = toutL;
        xout = xout';
        %xout = xout;
        wingspan = 2.04*ones(num_ac,1);
        flight_movie
        axis off
        title('')
        %Plot6(1:12,{toutL},xoutL,1,'m',0);
        %pause
       end %%if abs() > 1e-6
    end %%while nn
  end %%if MOVIES
end %For files

