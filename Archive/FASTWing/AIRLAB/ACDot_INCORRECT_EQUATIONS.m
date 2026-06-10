function [state_dot,controls] = ACdot(t,state,e)
%%%%% Equations of motion for a stable fixed wing aircraft.
%%%%%% Unwrap state and control vectors
global HP P M N K KCAB KCA timestepD nominal ICONTROL de da dr dT
global UCOMMAND ZCOMMAND MSLOPE BINTERCEPT ARCLENGTHPARENT
global XYZ LMN XYZP LMNP AEROMODEL

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

y0 = nominal(2);
z0 = nominal(3);
phi0 = nominal(4);
theta0 = nominal(5);
psi0 = nominal(6);
u0 = nominal(7);
v0 = nominal(8);
p0 = nominal(10);
q0 = nominal(11);
r0 = nominal(12);

mpcstate = state-nominal;
mpcstate = [mpcstate(1:3);u0;mpcstate(4:end)];

%%% Mass Moment of Inertia Matrix in Body Axis (slug.ft^2)
Ix = 0.4923;
Iy = 0.9546;
Iz = 1.3766;
I_b = [Ix 0 0;0 Iy 0;0 0 Iz];
I_b_inv = [1/Ix 0 0;0 1/Iy 0;0 0 1/Iz];
grav = 9.81;
mass = 55.07/grav;

% %%% Longitudinal Coefficients

% C_m_0 = 0.0598;
% C_L_0 = 0.062;
% C_D_0 = 0.028;
% C_L_alpha = 5.195;
% C_D_alpha = 1.3537;
% C_m_alpha = -0.9317;
% C_m_alpha_dot = -2.897;
% C_L_q = 4.589;
% C_m_q = -5.263;
% C_L_de = 0.2167;
% C_m_de = -0.8551;
% C_x_dT =  7.6509;
% C_D_u = 0;
% C_m_u = 0;

%%% Lateral Coefficients

% C_y_beta = -0.2083;
% C_l_beta = -0.0377;
% C_n_beta = 0.0116;
% C_l_p = -0.4625;
% C_n_p = -0.0076;
% C_l_r = 0.0288;
% C_n_r = -0.0276;
% C_l_da = -0.2559;
% C_n_da = -0.0216;
% C_y_dr = 0.1096;
% C_l_dr = 0.0085;
% C_n_dr = 0.0035;
% C_y_p = 0.0057;
C_y_r = 0.0645;

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
cp = cos(phi);
sp = sin(phi);
cs = cos(psi);
ss = sin(psi);

%%% Kinematics
L1 = [1 0 0; 0 cp sp; 0 -sp cp];
L2 = [ct 0 -st; 0 1 0; st 0 ct];
L3 = [cs ss 0; -ss cs 0; 0 0 1];
TH = (L1*L2*L3)';
xyz_dot = TH*[u; v; w];
xdot = xyz_dot(1);
ydot = xyz_dot(2);
zdot = xyz_dot(3);
pts_dot = [1, sp*st/ct, cp*st/ct; 0, cp, -sp; 0, sp/ct, cp/ct]*[p; q; r];

%%% Definitions of aero components
% uvwwind = [0;0;0];
% uvwwindbody = TH'*uvwwind;
% uaero = u + uvwwindbody(1);
% vaero = v + uvwwindbody(2);
% waero = w + uvwwindbody(3);

% % Total airspeed
% Vaero = norm([uaero vaero waero]);

% % Dynamic Pressure
% rho = 1.22566578494891*pow((1.00000000-0.0000225696709*abs(-200)),4.258);
% Qa = 1/2*rho*Vaero^2;

% % Angle of attack:
% if abs(u)>0
%     alpha = atan2(waero,uaero);
% else
%     alpha = 0;
% end

% % Sideslip
% if abs(Vaero)>0
%     beta = asin(vaero/Vaero);
% else
%     beta = 0;
% end

%%%%%%%%%%%PID%%%%%%%%%%%%
PID = 0;
NOMINALC = 1;
LMPC = 0;

%Initialize Controls
de = 0;
da = 0;
dT = 0;
dr = 0;

lbar0 = cos(MSLOPE);
lbar1 = sin(MSLOPE);
planedirection = lbar0*(x-0) + lbar1*(y-BINTERCEPT);
normAC = sqrt(x*x+y*y);
flightdirection = sign(xdot*lbar0+ydot*lbar1);
xs = 0 + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar0;
ys = BINTERCEPT + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar1;
ys = y;
zs = ZCOMMAND;

ICONTROL = ICONTROL + 1;
if LMPC && mod(ICONTROL,4)==3
  %disp('Computing Control')
  YCOMMAND = zeros(P*(HP),1);
  %%Now populate YCOMMAND
  xcoord0 = x;
  ycoord0 = y;
  zcoord0 = z;
  phicoord0 = phi;
  thetacoord0 = theta;
  p1 = sqrt((xcoord0-x)+sqrt(ycoord0-y)+(zcoord0-z));
  for ii = 0:HP-1
    scoord = 0 + (ii+1)/HP;
    xcoord1 = x + scoord*(xs-x);
    ycoord1 = y + scoord*(ys-y);
    zcoord1 = z + scoord*(zs-z);
    p2 = sqrt((xcoord1-x)^2+(ycoord1-y)^2+(zcoord1-z)^2);
    thetacoord1 = -atan2(zcoord1-zcoord0,p2-p1);
    psicoord = atan2(ycoord1-ycoord0,xcoord1-xcoord0);
    phicoord1 = -3*(psi-psicoord);
    if phicoord1 > 45*pi/180
      phicoord1 = sign(phicoord1)*45*pi/180;
    end
    %%Now use phicoord and thetacoord to generate p and q%%Now use phicoord and thetacoord to generate p and q
    pcoord = (phicoord1-phicoord0)/timestepD;
    qcoord = (thetacoord1-thetacoord0)/timestepD;
    ucoord = (p2-p1)/timestepD;
    ucoord = u0;
    %Reset
    p1 = p1;
    phicoord0 = phicoord1;
    thetacoord0 = thetacoord1;
    xcoord0 = xcoord1;
    ycoord0 = ycoord1;
    zcoord0 = zcoord1;
    %%Now substract nominal values to get deltas
    duii = ucoord - u0;
    dpii = pcoord - p0;
    dqii = qcoord - q0;
    YCOMMAND((ii)*P+1:(ii)*P+P,1)=[duii;dpii;dqii];
  end
  KCASTATE = KCA*mpcstate;
  YKCA = YCOMMAND-KCASTATE;
  U = K*(YKCA);
  uc = U(1:3);
  de = uc(1);
  da = uc(2);
  dT = uc(3);
  KV = 0.04;
  dr = -KV*v;
  %%%PID DEBUG
  %dT = 0.2*(UCOMMAND-u);
  %zerror = z-ZCOMMAND;
  %de = -2.0*zerror-0.3*zdot;
  % rcpath0 = xs - x;
  % rcpath1 = ys - y;
  % rcpathPsi0 = cs*rcpath0 + ss*rcpath1;
  % rcpathPsi1 = -ss*rcpath0 + cs*rcpath1;
  % vpcpathPsi0 = cos(MSLOPE)*xdot + sin(MSLOPE)*ydot;
  % vpcpathPsi1 = -sin(MSLOPE)*xdot + cos(MSLOPE)*ydot;
  % delpsi = -0.005*rcpathPsi1 + 0.02*vpcpathPsi1;
  % PSICOMMANDbody = psi - delpsi;
  % phipathcommand = -3*delpsi;
  % if (abs(phipathcommand) > 45*pi/180)
  %   phipathcommand = sign(phipathcommand)*45*pi/180;
  % end
  % da = 1.2*(phi-phipathcommand)+0.1*p;
end

if PID
  dT = 0.2*(UCOMMAND-u);
  zerror = z-ZCOMMAND;
  de = -2.0*zerror-0.3*zdot;
  lbar0 = cos(MSLOPE);
  lbar1 = sin(MSLOPE);
  planedirection = lbar0*(x-0) + lbar1*(y-BINTERCEPT);
  normAC = sqrt(x*x+y*y);
  flightdirection = sign(xdot*lbar0+ydot*lbar1);
  xs = 0 + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar0;
  ys = BINTERCEPT + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar1;
  rcpath0 = xs - x;
  rcpath1 = ys - y;
  rcpathPsi0 = cs*rcpath0 + ss*rcpath1;
  rcpathPsi1 = -ss*rcpath0 + cs*rcpath1;
  vpcpathPsi0 = cos(MSLOPE)*xdot + sin(MSLOPE)*ydot;
  vpcpathPsi1 = -sin(MSLOPE)*xdot + cos(MSLOPE)*ydot;
  delpsi = -0.005*rcpathPsi1 + 0.02*vpcpathPsi1;
  PSICOMMANDbody = psi - delpsi;
  phipathcommand = -3*delpsi;
  if (abs(phipathcommand) > 45*pi/180)
    phipathcommand = sign(phipathcommand)*45*pi/180;
  end
  da = 1.2*(phi-phipathcommand)+0.1*p;
  KV = 0.04;
  dr = -KV*v;
end

if NOMINALC
  da = 0;
  dr = 0;
  de = -0.06103;
  dT = 0;
end
if length(e) == 4 && e(4) ~= 42
  da = e(1)+da;
  dr = e(2)+dr;
  de = e(3)+de;
  dT = e(4)+dT;
end

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
% da = 0;
% de = 0;
% dr = 0;
% dT = 0;
controls = [da;dr;de;dT];


%%%CONTROL FORCE%%%

%%% Calculation of body forces

% C_L = C_L_0 + C_L_alpha*alpha + C_L_q*q + C_L_de*de;
% C_D = C_D_0 + C_D_alpha*(alpha^2) + C_D_u*u;

% C_l = C_l_beta*beta + C_l_p*p + C_l_r*r + C_l_da*da + C_l_dr*dr;
% C_m = C_m_0 + C_m_alpha*alpha + C_m_q*q + C_m_de*de + C_m_u*u;
% C_n = C_n_p*p + C_n_beta*beta + C_n_r*r + C_n_da*da + C_n_dr*dr;
% C_x = 0;
% C_y = C_y_beta*beta + C_y_dr*dr + C_y_p*p + C_y_r*r;
% C_z = 0;

% Lift = C_L*Qa*S;
% Drag = C_D*Qa*S;
% T0 = 5.0882;
% Thrust = T0 + C_x_dT*dT;

GW = mass*grav;
Xw = -GW*st;
%sp = 0;
%cp = 1; %something is going on with this?
%Find the bi-frication point and simulate from there.
Yw = GW*sp*ct;
Zw = GW*cp*ct;  

% Xa = Lift*sin(alpha) - Drag*cos(alpha) + C_x*Qa*S  + Thrust;
% Ya = C_y*Qa*S;
% Za = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Qa*S;
% La = C_l*Qa*S*c_bar;
% Ma = C_m*Qa*S*c_bar;
% Na = C_n*Qa*S*c_bar;

%%Point Aero Model
if AEROMODEL
  [XYZa,LMNa,Bkdn] = Panels(state,controls);
else
  [XYZa,LMNa] = Point(state,controls);
end
%pause
XYZ = XYZa;
LMN = LMNa;
XYZP = XYZa;
LMNP = LMNa;

X = XYZa(1) + Xw;
Y = XYZa(2) + Yw;
Z = XYZa(3) + Zw;

L = LMNa(1);
M = LMNa(2);
N = LMNa(3);

%%% Dynamics
uvw_dot = [X; Y; Z]/mass + [r*v-q*w;p*w-r*u;q*u-p*v];
pqr_dot = I_b_inv*([L; M; N]-[0 -r q;r 0 -p;-q p 0]*I_b*[p; q; r]);

%%% Collect Derivatives

%xyz_dot(1) = 0;
%xyz_dot(2) = 0;
%xyz_dot(3) = 0;

%pts_dot(1) = 0; %phi
%pts_dot(2) = 0; %theta
%pts_dot(3) = 0; %psi

%uvw_dot(1) = 0; %u
%uvw_dot(2) = 0; %v
%uvw_dot(3) = 0; %w

%pqr_dot(1) = 0; %p
%pqr_dot(2) = 0; %q
%pqr_dot(3) = 0; %r

state_dot = [xyz_dot; pts_dot; uvw_dot; pqr_dot];
%t
% if mod(ICONTROL,4)==3
%     U
% end
% pause

