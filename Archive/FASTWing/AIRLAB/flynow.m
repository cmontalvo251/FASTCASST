%purge
close all

global HP P M N K KCAB KCA timestepD A B nominal control0 ICONTROL
global UCOMMAND ZCOMMAND MSLOPE BINTERCEPT ARCLENGTHPARENT de da dr dT
global XYZ LMN XYZP LMNP AEROMODEL

%%%%COMPUTE CONTROL%%%

UCOMMAND = 20;
ZCOMMAND = -200;
MSLOPE = 0;
BINTERCEPT = 0;
ARCLENGTHPARENT = 10;

%%Admin Flags
STATES = 1; %plot states
CONTROLS = 0;

SIXDOF = 1;
LMODEL = 0;
DMODEL = 0;

%%Time Stuff
% t0 = 7.2255;
% tf = 8.5;
t0 = 0;
tf = 20;
timestep = 0.001;
timestepD = 0.01;
MPCMatrices;

%%Initial Conditions
%x0 = [-50 4 -200  0.2 .3 0.1  20 8 9    10 -1 3]'; %%SI UNITS
x0 = [0 0 -200 0 0 0 23 -1 0 0 0 -2]';
%x0 = [0 0 -200 0 0 0 21 9 0 0 0 -1.5]';
%x0 = [0 0 -200 0 0 0 17 13 0 0 0 -1]';
% x0 = [155.410334;
% -3.712300 ;
% -188.409813 ;
% -0.036825 ;
% -0.019750 ;
% -0.366853 ;
% 21.977583 ;
% 5.894365 ;
% 1.455328 ;
% -0.018060 ; 
% -0.005544 ;
% -0.768720 ];
nominal = [-50;0;-200;0;0.0560096547756443;0;20.192;0;1.13214709899632;0;0;0];
%x0 = nominal;
control0 = [0;0;-0.06103;0];
ICONTROL = 2;
%%%%6DOF%%%
if SIXDOF
  disp('6DOF Model')
  %nominal = [0;0;-200;0;0.0560096547756443;0;20.192;0;1.13214709899632;0;0;0];
  AEROMODEL = 1;
  [toutSM,xoutSM,uoutSM] = odeK4(@ACdot,[t0 tf],x0,timestep,[0;0;0;42],1);
  AEROMODEL = 0;
  [tout6,xout6,uout6] = odeK4(@ACdot,[t0 tf],x0,timestep,[0;0;0;42],1);
  clc
  % XYZvec = zeros(3,length(tout6));
  % LMNvec = XYZvec;
  % XYZPvec = XYZvec;
  % LMNPvec = XYZvec;
  % for ii = 1:length(tout6)
  %   AEROMODEL = 1;
  %   [state_dot,controls] = ACdot(tout6(ii),xout6(:,ii),[0;0;0;42]);
  %   XYZPvec(:,ii) = XYZP;
  %   LMNPvec(:,ii) = LMNP;
  %   AEROMODEL = 0;
  %   [state_dot,controls] = ACdot(tout6(ii),xout6(:,ii),[0;0;0;42]);
  %   XYZvec(:,ii) = XYZ;
  %   LMNvec(:,ii) = LMN;
    
  % end
end
if LMODEL
  disp('Linear Model')
  %%%%%%LMODEL%%%%%%%%
  %%Linear model initial conditions
  x0l = x0-nominal;
  %%Augment x0l with nominal velocity
  x0l = [x0l(1:3);nominal(7);x0l(4:end)];
  [toutLTI,xoutLTI,uoutLTI] = odeK4(@LinearM,[t0 tf],x0l,timestep,[0;0;0;42],1);
  %%First strip out u0
  xoutLTI(4,:) = [];
  %%Then augment state with nominal
  [r,c] = size(xoutLTI);
  xoutLTI = xoutLTI + nominal*ones(1,c);
  clc
end
if DMODEL
  disp('Discrete Model')
  %%%%%%%DMODEL%%%%%%%%
  %%Linear model initial conditions
  x0l = x0-nominal;
  %%Augment x0l with nominal velocity
  x0l = [x0l(1:3);nominal(7);x0l(4:end)];
  [toutD,xoutD,uoutD] = olde(@LinearD,[t0 tf],x0l,control0,timestepD,10);
  %%First strip out u0
  xoutD(4,:) = [];
  %%Then augment state with nominal
  [r,c] = size(xoutD);
  xoutD = xoutD + nominal*ones(1,c);
end

Az = 37;
El = 14;
wingspan = 2.04;
num_ac = 1;
ts = 0.1;
NSTATES = 12;

PlotRoutine
%flight_movie

