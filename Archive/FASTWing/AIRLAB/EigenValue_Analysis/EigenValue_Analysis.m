%purge
global wdot udot

format long g
%%First we start with the Modes computed from Nonlinear Simulation
files = {'Single_Aircraft_Single_Computation_Point.txt'};
Alinear = dlmread(['Linear_Matrices/',files{1}]);
[r,c] = size(Alinear);
NSTATE = 15;
num_ac = round(r/NSTATE);

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
acmodes = eig(Alinear);
[eigenvectors,singularvalues] = eig(Alinear);

xnominal = dlmread(['Linear_Matrices/',files{1}(1:end-4),'.nom']);
xnominal = xnominal(:,1);
xnominal(delrows)=[];
%Trim Control

%%%Small UAV
dT = 0.003221;
da = 0;
dr = 0;
de = 0.011753;
%%%Etkins book
% dT = 0;
% da = 0;
% dr = 0;
% de = 0;
% Ve = 733/3.28; %%ft/s converted to m/s
%%Emily's A/C
% x = 4.843664078000000e+003;
% y = -28.849928100000000;
% z = -100;
% phi = -5.618263395821422e-012;
% theta = 0.065764460337741;
% psi =-0.012046091184949;
% u =  17.262602539996674;
% v =  -0.000000212062166;
% w =   1.136905237997596;
% p = 2.887674176000000e-006;
% q = 4.509198856000000e-010;
% r = -4.384153917000000e-005;
% dT = 0.450434257300000;
% de = 0.014626909840000;
% dr = -3.034620045000000e-004;
% da = 0.057378449650000;
% xnominal = [x;y;z;phi;theta;psi;u;v;w;p;q;r];

controls = [de,da,dT,dr];
[state_dot0,controls] = SixDOF_Derivative(0,xnominal,controls);

x0 = xnominal;

Am = zeros(12,12);
del = 1e-8;
for jj = 1:length(x0)
  statep = x0;
  statep(jj) = statep(jj) + del;
  [statedot,controls] = SixDOF_Derivative(0,statep,controls);
  Am(:,jj) = (statedot-state_dot0)./(del);
end

matlabmodes = eig(Am)


%ADDED INDUCED VELOCITY to DRAG ON WING
%%Added induced angle of attack on horizontal tail
%%Added induced velotiy to drag on tail
%%Making computation of horiontal and vertical tail the same
%%Changed Fuselage Aero Routine

