clear
clc

%%%%1D Example
% m = 2;
% A = [0 1; 0 0];
% B = [0 1/m]';

%%%2D Example
% A = [0 1 0 0; 0 0 0 0;0 0 0 1;0 0 0 0];
% B = [0 0 ; 1/m 0 ; 0 0 ;0 1/m];

%%%Dubin's Aircraft
% V = 20;
% Izz = 5;
% A = [0 0 0 1 0; 0 0 V 0 0;0 0 0 0 1;0 0 0 0 0;0 0 0 0 0];
% B = [0 0; 0 0; 0 0; 1/m 0;0 1/Izz];

%%%Dubin's Aircraft with Sliding
% mbar = -2;
% A = [0 0 0 1 0 0;0 0 V 0 1 0;0 0 0 0 0 1;0 0 0 mbar 0 0; 0 0 0 0 mbar 0; 0 0 0 0 0 0];
% B = [0 0;0 0;0 0; 1/m 0 ; 0 0; 0 1/Izz];

%%%3D Dubin's Aircraft
% Iyy = 3;
% subA = [0 0; 0 V; -V 0];
% A = [zeros(3,3),subA,eye(3,3),zeros(3,2);zeros(2,8),eye(2,2);zeros(3,5),mbar*eye(3,3),zeros(3,2);zeros(2,10)];
% B = [zeros(5,3);1/m 0 0;zeros(2,3);0 1/Iyy 0;0 0 1/Izz];

%%%Simplified Aircraft Model
% Ixx = 2;
% subA = [0 0 0;0 0 V;0 -V 0];
% O33 = zeros(3,3);
% I33 = eye(3,3);
% A = [O33,subA,I33,O33;O33,O33,O33,I33;O33,O33,mbar*I33,O33;O33,O33,O33,O33];
% B = [zeros(6,4);0 0 0 1/m;zeros(2,4);0 1/Izz 0 0;1/Iyy 0 0 0;0 0 1/Ixx 0];

%%%%Simplified Aircraft Model with Gravity
% g = 9.81;
% g33 = [0 -g 0;g 0 0; 0 0 0];
% m33 = mbar*I33;
% CL = 2.6;
% m33(3,1) = -CL;
% u33 = [0 0 0;0 0 -V;0 V 0];
% A = [O33,subA,I33,O33;O33,O33,O33,I33;O33,g33,m33,u33;O33,O33,O33,O33];
% disp(A)
% B = [zeros(6,4);0 0 0 1/m;zeros(2,4);0 1/Izz 0 0;1/Iyy 0 0 0;0 0 1/Ixx 0];

%%%%Simplified Aircraft Model with Gravity and Extra Aero
%uw = 0.44;
%m33(1,3) = uw;
%u33(2,1) = 1.09;
%A = [O33,subA,I33,O33;O33,O33,O33,I33;O33,g33,m33,u33;O33,O33,O33,O33]
%B = [zeros(6,4);0 0 0 1/m;zeros(2,4);0 1/Izz 0 0;1/Iyy 0 0 0;0 0 1/Ixx 0];

%%%%Full Linearized Aircraft Model
NoStates = 15;
RUNCODE = 1;
COMPILE = 1;
MOVIES = 0;
timestep = 0.001;
ts = timestep*10;
tend = 0.5;
files = {'AC_Jung.txt','AC_Phillips_SI.txt'};

for ii = 1:length(files)
  data = dlmread(['/home/carlos/Documents/PhD/Meta-Aircraft/MultiMMACSV10/source/Out_Files/Linearized/',files{ii}]);
  [r,c] = size(data);
  Alinear = data(1:c,1:c);
  [r,c] = size(Alinear);
  num_ac = round(r/NoStates);
  Blinear = data(NoStates*num_ac+1:end,1:num_ac*4);
  Alinear = Alinear(1:(num_ac*NoStates),1:(num_ac*NoStates));

  %%%%Remove xint,yint and zint
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
  if ii == 1
    A = Alinear;
    B = Blinear;
  else
    Ap = Alinear;
    Bp = Blinear;
  end
end

%[Ap,Bp] = Phillips_Controllability;

%%%%Lateral Affects (vdot = f(p) or C_y_p)
%%%Assume Cyp = 0
%Alinear(8,10) = 0;
%%%Longitudinal Affects (qdot = f(u) or C_m_u)
%%%Let C_m_u = 0
%Alinear(11,7) = 0;

%%%Control affects
%%rdot = f(da,dr)
%%%Cndr is negative for phillips and negative for my aircraft
%%%Cnda is positive for phillips and negative for my aircraft
%Blinear(12,[2 3]) = -Blinear(12,[2 3]);

%%%%At this point all rows have the same sign however the magnitude is off.
%%%%Lets start to vary the magnitudes until we get something that works. Or
%%%%we could try and dimensionalize the values or non-dimensionalize our
%%%%values

%%%Remove x,y,z and psi
%Alinear([1 2 3 6],:) = [];
%Alinear(:,[1 2 3 6],:) = [];
%Blinear([1 2 3 6],:) = [];

%%%Convert to Simplified Aero Model
%Blinear(8,3) = 0; %%%dv/d(dr)
%Blinear(9,1) = 0; %%%dw/d(de)
%Blinear(10,3) = 0; %%%%dp/d(dr)
%Blinear(12,2) = 0; %%%dr/d(da)

%%%%These values are almost zero
%Alinear(1,5) = 0;
%Alinear(1,6) = 0;
%Alinear(9,4) = 0;
%Alinear(9,6) = 0;

%%%%%Rotational Damping
%Alinear(10,10) = 0; %%%dpdot/dp

%%%Gravity
%Alinear(7,5) = 0; %%%du/dtheta
%Alinear(8,4) = 0; %%%dv/dphi
%Alinear(9,5) = 0; %%%dw/dtheta

%%%%V and W Derivatives
%Alinear(8,10) = 0; %%%dv/dp
%Alinear(8,12) = 0; %%%dv/dr
%Alinear(9,7) = 0; %%%dw/du
%Alinear(9,11) = 0; %%%dw/dq

%%%%%Other Parameters
%Alinear(7,9) = 0; %%%du/dw
%Alinear(7,11) = 0; %%%du/dq
%Alinear(10,8) = 0; %%%dp/dv
%Alinear(10,12) = 0; %%%dp/dr

%Alinear(11,7) = 0; %%%dq/du
%Alinear(11,9) = 0; %%%dq/dw
%Alinear(11,11) = 0; %%%dqdot/dq

%Alinear(12,8) = 0; %%%dr/dv
%Alinear(12,10) = 0; %%%dr/dp
%Alinear(12,12) = 0; %%%drdot/dr

%disp(Alinear)

%%%%Test for controllability
[v,s] = eig(A);
[w,s2] = eig(A');
s1 = eig(A);
%wc = controllability(A,B);
wB = w'*B;

%%%%Remove complex conjugate pairs
%wB([2 5 7],:) = [];

%%%%Remove last 4 rows
%wB([6 7 8 9],:) = []

%r = rank(wc);
%disp(r)
[N,d] = size(A);

for ii = 1:N
    if sum(abs(w(:,ii)'*B)) == 0
        disp(['Mode ',num2str(ii),' is uncontrollable'])
    end
end
