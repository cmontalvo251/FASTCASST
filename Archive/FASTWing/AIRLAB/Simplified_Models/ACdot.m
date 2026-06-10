function [state_dot,controls] = ACdot(t,state,e)
%%% Equations of motion for a stable fixed wing aircraft.
%%% Unwrap state and control vectors

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
C_L_alpha = 5.195;
C_D_alpha = 1.3537;
C_m_alpha = -0.9317;
C_m_alpha_dot = -2.897;
C_L_q = 4.589;
C_m_q = -5.263;
C_L_de = 0.2167;
C_m_de = -0.8551;
C_x_dT = 10;

%%% Lateral Coefficients

C_y_beta = -0.2083;
C_l_beta = -0.0377;
C_n_beta = 0.0116;
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
uvwwind = [0;0;0];
uvwwindbody = TH'*uvwwind;
uaero = u + uvwwindbody(1);
vaero = v + uvwwindbody(2);
waero = w + uvwwindbody(3);

% Total airspeed
Vaero = norm([uaero vaero waero]);

% Dynamic Pressure
rho = 1.22566578494891*pow((1.00000000-0.0000225696709*abs(z)),4.258);
Q = 1/2*rho*Vaero^2;

% Angle of attack:
if abs(u)>0
    alpha = atan2(waero,uaero);
else
    alpha = 0;
end

% Sideslip
if abs(Vaero)>0
    beta = asin(vaero/Vaero);
else
    beta = 0;
end

%%% Calculation of body forces

C_L = C_L_0 + C_L_alpha*alpha + C_L_q*q;
C_D = C_D_0 + C_D_alpha*(alpha^2);

C_l = C_l_beta*beta + C_l_p*p + C_l_r*r;
C_m = C_m_alpha*alpha + C_m_q*q;
C_n = C_n_p*p + C_n_beta*beta + C_n_r*r;
C_x = 0;
C_y = C_y_beta*beta;
C_z = 0;

Lift = C_L*Q*S;
Drag = C_D*Q*S;
T0 = 5.0882;
Thrust = T0;

GW = mass*grav;
Xw = -GW*st;
Yw = GW*sp*ct;
Zw = GW*cp*ct;  

X0 = Lift*sin(alpha) - Drag*cos(alpha) + C_x*Q*S  + Thrust;
Y0 = C_y*Q*S;
Z0 = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Q*S;
L0 = C_l*Q*S*c_bar;
M0 = C_m*Q*S*c_bar;
N0 = C_n*Q*S*c_bar;

%%%%COMPUTE CONTROL%%%

UCOMMAND = Vtrim;
ZCOMMAND = -200;
MSLOPE = 0;
BINTERCEPT = 0;
ARCLENGTHPARENT = 10;

%%%%%%%%%%%PID%%%%%%%%%%%%
PID = 0;
FLINEAR = 0;
NOMINAL = 1;

G = (1/mass).*[0 0 Q*S*C_L_de*sin(alpha) mass*C_x_dT;0 Q*S*C_y_dr 0 0;0 0 -Q*S* ...
	       C_L_de*cos(alpha) 0;Q*S*c_bar*C_l_da Q*S*c_bar*C_l_dr 0 0; 0 ...
	       0 Q*S*c_bar*C_m_de 0;Q*S*c_bar*C_n_da Q*S*c_bar*C_n_dr 0 0];
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

if FLINEAR
  F = [0;0;0;0;0;0];
  F(1:3) = [X0; Y0; Z0]/mass + [r*v-q*w;p*w-r*u;q*u-p*v];
  F(4:6) = I_b_inv*([L0; M0; N0]-[0 -r q;r 0 -p;-q p 0]*I_b*[p; q; r]);
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
  flg = [0.2*(UCOMMAND-u);-0.05*rcpathPsi1+0.1*vpcpathPsi1;-2.0*(z-ZCOMMAND)-5.2*zdot;0;0;0];
  controls = inv(G'*G)*G'*(F-flg);
  da = controls(1);
  dr = controls(2);
  de = controls(3);
  dT = controls(4);
end
if NOMINAL
  da = 0;
  dr = 0;
  de = 0;
  dT = 0;
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
controls = [da;dr;de;dT];

%%%CONTROL FORCE%%%

% Xc = Q*S*C_L_de*de*sin(alpha) + C_x_dT*dT;
% Yc = Q*S*C_y_dr*dr;
% Zc = -Q*S*C_L_de*de*cos(alpha);
% Lc = Q*S*c_bar*(C_l_da*da + C_l_dr*dr);
% Mc = Q*S*c_bar*(C_m_de*de);
% Nc = Q*S*c_bar*(C_n_da*da+C_n_dr*dr);
CF = G*controls*mass;
Xc = CF(1);
Yc = CF(2);
Zc = CF(3);
Lc = CF(4);
Mc = CF(5);
Nc = CF(6);

%%Add all forces
X = X0 + Xw + Xc;
Y = Y0 + Yw + Yc;
Z = Z0 + Zw + Zc;

L = L0 + Lc;
M = M0 + Mc;
N = N0 + Nc;

%% Dynamics
uvw_dot = [X; Y; Z]/mass + [r*v-q*w;p*w-r*u;q*u-p*v];
pqr_dot = I_b_inv*([L; M; N]-[0 -r q;r 0 -p;-q p 0]*I_b*[p; q; r]);

%% Collect Derivatives

state_dot = [xyz_dot; pts_dot; uvw_dot; pqr_dot];

