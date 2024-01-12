function [state_dot,controls] = TwoD(t,state,e)
%%% Equations of motion for a stable fixed wing aircraft.
%%% Unwrap state and control vectors
global HP P M N K KCA KCAB timestepD

x = state(1);
y = state(2);
z = state(3);
phi = state(4);
theta = state(5);
psi = state(6);
u = state(7);
v = state(8);
w = state(9);
p = state(10);
q = state(11);
r = state(12);

%%% Mass Moment of Inertia Matrix in Body Axis (slug.ft^2)
Ix = 0.4923;
Iy = 0.9546;
Iz = 1.3766;
I_b = [Ix 0 0;0 Iy 0;0 0 Iz];
I_b_inv = [1/Ix 0 0;0 1/Iy 0;0 0 1/Iz];
grav = 9.81;
mass = 55.07/grav;

%%% Longitudinal Coefficients

C_L_0 = 0.062;
C_D_0 = 0.028;
C_L_q = 4.589;
C_m_q = -5.263;
%C_m_q = -0.5263;
C_L_de = 0.2167;
C_m_de = -0.8551;
C_x_dT = 10;

%%% Lateral Coefficients

C_l_p = -0.4625;
C_n_p = -0.0076;
C_l_r = 0.0288;
C_n_r = -0.0276;
C_l_da = -0.2559;
C_n_da = -0.0216;
C_y_dr = 0.1096;
C_l_dr = 0.0085;
C_n_dr = 0.0035;

%%z Geometry and Inertias

% Reference Area (m^2)
S = 0.6558;
% Wingspan (m)
b = 2.04;
%Mean chord
c_bar = 0.3215;
%Trim Velocity
Vtrim = 20;

%%% Trig functions
ct = cos(theta);
st = sin(theta);
cp = 1;
sp = 0;
cs = cos(psi);
ss = sin(psi);

%%% Kinematics
L2 = [ct 0 -st; 0 1 0; st 0 ct];
L3 = [cs ss 0; -ss cs 0; 0 0 1];
TH = (L2*L3)';
xyz_dot = TH*[u; 0; 0];
xdot = xyz_dot(1);
ydot = xyz_dot(2);
zdot = xyz_dot(3);
pts_dot = [0; q; r];
thetadot = pts_dot(2);
psidot = pts_dot(3);

%%% Definitions of aero components
uaero = u;
vaero = v;
waero = w;

% Total airspeed
Vaero = sqrt(uaero^2+vaero^2+waero^2);

% Dynamic Pressure
rho = 1.22566578494891;
Qa = 1/2*rho*Vaero^2;

alpha = 0;
beta = 0;

%%% Calculation of body forces

C_D = C_D_0;

C_m = C_m_q*q;
C_n = C_n_r*r;

Drag = C_D*Qa*S;
Thrust = 5.0882;

X0 = -Drag + Thrust;
M0 = C_m*Qa*S*c_bar;
N0 = C_n*Qa*S*c_bar;

%%%%%%%%%MAPPING%%%%%%%

MSLOPE = 0;
MSLOPE2 = 0;
BINTERCEPT2 = 10;
BINTERCEPT = 0;
ARCLENGTHPARENT = timestepD*Vtrim*HP;
lbar(1) = cos(MSLOPE);
lbar(2) = sin(MSLOPE);
planedirection = lbar(1)*(x-0) + lbar(2)*(y-BINTERCEPT);
normAC = sqrt(x*x+y*y);
flightdirection = sign(xdot*lbar(1)+ydot*lbar(2));
xs = 0 + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar(1);
ys = BINTERCEPT + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar(2);
zs = -200-BINTERCEPT2;
rwpI = [xs - x;ys - y;zs - z];
rwpB = TH'*rwpI;
vwpB(1) = cos(MSLOPE)*xdot + sin(MSLOPE)*ydot;
vwpB(2) = -sin(MSLOPE)*xdot + cos(MSLOPE)*ydot;
vwpB(3) = sin(theta)*u;
delpsi = 0.005*(rwpB(2)) - 0.02*(vwpB(2));
thetac = -0.005*(rwpB(3)) + 0.02*(vwpB(3));
uc = 0.9*(rwpB(1))+0.5*(vwpB(1));

%%%%COMPUTE CONTROL%%%
%%%%%%%%%%%%%%%%%%%%%%%
PID = 0;
FLINEAR = 0;
NOMINAL = 0;
SLIDING = 1;
LMPC = 0;

if PID
  qc = 1*(thetac-theta)+0*(-thetadot);
  rc = 1*(delpsi)+0*(-psidot);
  uc = 20;
  dT = 1*(uc-u);
  de = -5*(qc-q);
  dr = 5*(rc-r);
end
if FLINEAR
  gtp = -1*([theta-thetac;-delpsi]);
  qcrc = gtp;
  gqr = [-50 0;0 -5]*([q;r]-qcrc);
  F = I_b_inv*([0; M0; N0]-[0 -r q;r 0 -0;-q 0 0]*I_b*[0; q; r]);
  f = [F(2);F(3)];
  g = [Qa*S*c_bar*C_m_de 0;0 Qa*S*c_bar*C_n_dr];
  dedr = inv(g)*(gqr-f);
  de = dedr(1);
  dr = dedr(2);
  uc = 30;
  gu = -1*(u-uc);
  fu = X0/mass+Qa*S*C_L_de*sin(alpha)*de;
  dT = (1/C_x_dT)*(gu-fu);
end
if LMPC
  YCOMMAND = zeros(P*(HP+1),1);
  %%%%GENERATE PATHS from x,y,z to xs,ys,zs%%
  % invC = [3 -1;-2 1];
  % c0 = x;
  % c1 = xdot;
  % d0 = y;
  % d1 = ydot;
  % xdots = 20;
  % ydots = 0;
  % c2c3 = invC*[xs-c0-c1;xdots-c1];
  % c2 = c2c3(1);
  % c3 = c2c3(2);
  % d2d3 = invC*[ys-d0-d1;ydots-d1];
  % d2 = d2d3(1);
  % d3 = d2d3(2);
  scoord = linspace(0,1,HP+1);
  % xcoord = c0 + c1.*scoord + c2.*scoord.^2 + c3.*scoord.^3;
  % ycoord = (d0 + d1.*scoord + d2.*scoord.^2 + d3.*scoord.^3);
  xcoord = x + scoord.*(xs-x);
  ycoord = y + scoord.*(ys-y);
  zcoord = z + scoord.*(zs-z);
  ucoord = 0.*zcoord;
  thetacoord = 0.*ucoord;
  psicoord = 0.*thetacoord;
  qcoord = 0.*thetacoord;
  rcoord = 0.*thetacoord;
  %%%%Use x,y,z path to compute u,theta and psi
  ucoord(1) = u;
  thetacoord(1) = theta;
  psicoord(1) = psi;
  qcoord(1) = q;
  rcoord(1) = r;
  YCOMMAND(1:3,1) = [u;q;r];
  for ii = 2:length(xcoord)
    p0 = sqrt((xcoord(ii-1)-xcoord(1))^2+(ycoord(ii-1)-ycoord(1))^2+(zcoord(ii-1)-zcoord(1))^2);
    p1 = sqrt((xcoord(ii)-xcoord(1))^2+(ycoord(ii)-ycoord(1))^2+(zcoord(ii)-zcoord(1))^2);
    ucoord(ii) = (p1-p0)/timestepD;
    ucoord(ii) = Vtrim;
    thetacoord(ii) = -atan2(zcoord(ii)-zcoord(ii-1),p1-p0);
    psicoord(ii) = atan2(ycoord(ii)-ycoord(ii-1),xcoord(ii)-xcoord(ii-1));
    %%Now use psicoord and thetacoord to generate q and r    
    qcoord(ii) = (thetacoord(ii)-thetacoord(ii-1))/timestepD;
    rcoord(ii) = (psicoord(ii)-psicoord(ii-1))/timestepD;
    %%Now populate YCOMMAND
    YCOMMAND((ii-1)*P+1:(ii-1)*P+P,1)=[ucoord(ii);qcoord(ii);rcoord(ii)];
  end
  YCOMMAND(1:3) = [];
  statek = [x;y;z;theta;psi;u;q;r];
  U = K*(YCOMMAND-KCA*statek);
  uc = U(1:3);
  de = uc(1);
  dr = uc(2);
  dT = uc(3);
end
if SLIDING
  %%Option 1 - 1 Sliding Surface
  Ys = [theta;psi];
  Yc = [thetac;psi+delpsi];
  Ysdot = [thetadot;psidot];
  Ycdbldot = [0;0];%thetadbldotc,psidbldotc
  lambda = [1 0;0 0.1];
  eta = [1 0;0 1];
  PHI = [0.00001 1];
  %%Look at the Feedback linearization stuff for g and Fc
  F = I_b_inv*([0; M0; N0]-[0 -r q;r 0 -0;-q 0 0]*I_b*[0; q; r]);
  Fc = [F(2);F(3)];
  gc = [Qa*S*c_bar*C_m_de 0;0 Qa*S*c_bar*C_n_dr];
  Surface = (Ysdot-Ycdot)+lambda*(Ys-Yc);
  %dedr = inv(gc)*(Ycdbldot-Fc-lambda*(Ysdot-Ycdot)-eta*sign(Surface));
  dedr = inv(gc)*(Ycdbldot-Fc-lambda*(Ysdot-Ycdot)-eta*[sat(Surface(1)/PHI(1));sat(Surface(2)/PHI(2))]);
  de = dedr(1);
  dr = dedr(2);
  uc = 20;
  dT = 1*(uc-u);
  %%Option 2 - 2 sliding surfaces
end
if NOMINAL
  if length(e) == 4
    da = e(1);
    dr = e(2);
    de = e(3);
    dT = e(4);
  else
    da = 0;
    dr = 0;
    de = 0;
    dT = 0;
  end
end
da = 0;
G = (1/mass).*[0 0 0 mass*C_x_dT;0 0 0 0;0 0 -Qa*S*C_L_de 0;Qa*S*c_bar*C_l_da Qa*S*c_bar*C_l_dr 0 0; 0 0 Qa*S*c_bar*C_m_de 0;Qa*S*c_bar*C_n_da Qa*S*c_bar*C_n_dr 0 0];

%%%LIMIT CONTROLS%%%%

if abs(de) > 30*pi/180
  de = sign(de)*30*pi/180;
end
if abs(dr) > 30*pi/180
  dr = sign(dr)*30*pi/180;
end
if abs(da) > 30*pi/180
  da = sign(da)*30*pi/180;
end
controls = [da;dr;de;dT];

%%%CONTROL FORCE%%%

% Xc = Qa*S*C_L_de*de*sin(alpha) + C_x_dT*dT;
% Yc = Qa*S*C_y_dr*dr;
% Zc = -Qa*S*C_L_de*de*cos(alpha);
% Lc = Qa*S*c_bar*(C_l_da*da + C_l_dr*dr);
% Mc = Qa*S*c_bar*(C_m_de*de);
% Nc = Qa*S*c_bar*(C_n_da*da+C_n_dr*dr);
CF = G*controls*mass;
Xc = CF(1);
Mc = CF(5);
Nc = CF(6);

%%Add all forces
X = X0 + Xc;
M = M0 + Mc;
N = N0 + Nc;

%% Dynamics
uvw_dot = [X; 0; 0]/mass;
pqr_dot = I_b_inv*([0; M; N]-[0 -r q;r 0 -0;-q 0 0]*I_b*[0; q; r]);

%% Collect Derivatives
state_dot = [xyz_dot; pts_dot; uvw_dot; pqr_dot];

