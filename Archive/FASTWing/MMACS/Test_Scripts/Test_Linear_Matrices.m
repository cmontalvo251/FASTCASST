%purge

files = {'Single_Aircraft.txt'};
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

%%First add a column of zeros
A = Alinear;
B = Blinear;
l = 12;
A = [A(:,1:3),zeros(l,1),A(:,4:end)];
%%Now add a row of zeros
A = [A(1:3,:);zeros(1,l+1);A(4:end,:)];
%Finally change A(1,4) = 1 to account for u0
A(1,4) = 1;
%%For B we need to simply add a row of zeros
B = [zeros(1,3);B];
timestepDiscrete = 0.001;
Adi = expm(A*timestepDiscrete);
jump = 10;
Adjump = Adi^jump;
Ap = A(8:13,8:13);
Adp = Adi(8:13,8:13);
Bp = B(8:13,:);
Bdp = inv(Ap)*(Adp-eye(6))*Bp;
Bdpjump = zeros(6,3);
for ii = 1:jump
  Bdpjump = Bdpjump + Adp^(ii-1)*Bdp;
end
Bdjump = [zeros(7,3);Bdpjump];
Adiscrete = Adjump;
Bdiscrete = Bdjump;


Adiscrete = [1.0000         0         0    0.0100         0   -0.0005         0    0.0099         0    0.0014         0   -0.0000         0;
      0    1.0000         0         0   -0.0262         0    0.1957         0    0.0100         0    0.0000         0    0.0000;
      0         0    1.0000         0         0   -0.1957         0   -0.0014         0    0.0097         0   -0.0001         0;
      0         0         0    1.0000         0         0         0         0         0         0         0         0         0;
      0         0         0         0    1.0000         0         0         0   -0.0000         0    0.0099         0    0.0015;
      0         0         0         0         0    1.0000         0    0.0000         0   -0.0002         0    0.0096         0;
      0         0         0         0    0.0000         0    1.0000         0    0.0000         0   -0.0000         0    0.0101;
      0         0         0         0         0   -0.0972         0    0.9991         0    0.0038         0   -0.0244         0;
      0         0         0         0    0.0971         0         0         0    0.9984         0    0.0274         0   -0.1920;
      0         0         0         0         0   -0.0129         0   -0.0045         0    0.9613         0    0.1730         0;
      0         0         0         0   -0.0002         0         0         0   -0.0049         0    0.9711         0    0.0212;
      0         0         0         0         0   -0.0000         0    0.0062         0   -0.0433         0    0.9179         0;
      0         0         0         0    0.0000         0         0         0    0.0005         0   -0.0061         0    0.9951];

Bdiscrete =[      0         0         0;
      0         0         0;
      0         0         0;
      0         0         0;
      0         0         0;
      0         0         0;
      0         0         0;
      0         0    0.0136;
      0   -0.0022         0;
-0.1320         0         0;
      0   -0.2764         0;
-0.7901         0         0;
      0   -0.0116         0];

%%dxdot = Alinear*dx + Blinear*du
xnominal = dlmread('source/Out_Files/Linearized/Single_Aircraft.nom');
xnominal = xnominal(1:NSTATE,1);
xnominal(delrows) = [];
xnominal(1) = 0;
dT = 1.228607;
de = -0.080052;
da = 0;
unominal = [de da dT]';

%%Initial conditions
xinitial =  [0;6;-200;0.0;0.0;0;20;0;0;0;0;0];
%xinitial =  [0;6;-197;0.0;0.136837;0;19.385697;0;2.669367;0;0;0];
uinitial = [-0.080052;0.0;1.228607];

%%Linear Simulation
global Ac Bc du0 Ad Bd
dx0 = xinitial - xnominal;
dx0 = [dx0(1:3);19.385697;dx0(4:end)];
du0 = uinitial - unominal;
Bd = Bdiscrete;
Ad = Adiscrete;
% Ac = Alinear;
% Bc = Blinear;
% [toutL,dxoutL] = odeK4(@LinearDerivs,[0 10],dx0,0.001,[],1);
[toutL,dxoutL,duoutL] = olde(@MPCController,[0 100],dx0,du0,timestepDiscrete*jump,10);

[r,c] = size(dxoutL);

xoutL = dxoutL + [xnominal(1:3);20;xnominal(4:end)]*ones(1,c);
xoutL(4,:) = [];

%%%Nonlinear Simulation
%system('rm Run.exe');
%system('~/Dropbox/BlackBox/compilec source/MMACSV10.cpp -w');
%system('./Run.exe source/Meta.ifiles');
%data = dlmread('source/Out_Files/Meta.OUT');
%xout = data(:,2:end);
%tout = data(:,1);

%Plot6([1:12],{toutL,tout},xoutL',1,'m',0,{'Discrete','NL'},{xout});

Plot6([1:12],{toutL},xoutL',1,'m',0,{'Discrete'});

plottool(1,'Controls',12,'Time(sec)','Control(rad)');
plot(toutL,duoutL')
legend('de','da','dT')
