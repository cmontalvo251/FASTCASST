%purge
close all

global HP P M N K KCAB KCA timestepD

%%Admin Flags
STATES = 1; %plot states
CONTROLS = 1;
SIXDOF = 1;
SMODEL = 0;
LMODEL = 0;
DMODEL = 0;

%%Time Stuff
t0 = 0;
tf = 100;
timestep = 0.001;

%%Initial Conditions
x0 = [0 0 -200    0 0 0  20 0 0    0 0 0]'; %%SI UNITS
u0 = [0;0;0;0];
%%%%6DOF%%%
if SIXDOF
  [tout6,xout6,uout6] = odeK4(@ACdot,[t0 tf],x0,timestep,[0;0;0;42],1);
end
%%%%SIMPLIFIED MODEL
if LMODEL || DMODEL || SMODEL
  timestepD = 0.1;
  MPCMatrices
end
if SMODEL
  [toutSM,xoutSM,uoutSM] = odeK4(@TwoD,[t0 tf],x0,timestep,[0;0;0;42],1);
end
%%%%LTI%%%%%%%%%
if LMODEL
  order = [1 2 3 5 6 7 11 12];
  x0l = x0(order);
  [toutLTI,xoutLTI,uoutLTI] = odeK4(@LinearM,[t0 tf],x0l,timestep,[0;0;0;42],1);
  %%pad with zeros
  [r,c] = size(xoutLTI);
  xoutLTI = [xoutLTI(1:3,:);zeros(1,c);xoutLTI(4:6,:);zeros(3,c);xoutLTI(7:8,:)];
  %%add nominal states
end
%%%%DISCRETE%MODEL%%%%
if DMODEL  
  order = [1 2 3 5 6 7 11 12];
  x0l = x0(order);
  [toutD,xoutD,uoutD] = olde(@LinearD,[t0 tf],x0l,u0,timestepD,1);
  %%pad with zeros
  [r,c] = size(xoutD);
  xoutD = [xoutD(1:3,:);zeros(1,c);xoutD(4:6,:);zeros(3,c);xoutD(7:8,:)];
end

Az = 37;
El = 14;
wingspan = 2.04;
num_ac = 1;
ts = 0.1;
NSTATES = 12;

PlotRoutine
%flight_movie

