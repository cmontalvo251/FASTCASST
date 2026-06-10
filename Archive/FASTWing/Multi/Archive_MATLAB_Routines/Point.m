function [XYZ,LMN,controls] = Point(state,controls,t,rcg2ac,kinematicderivs)
global S I_b c_bar b mass I_b_inv T0 num_ac

%%%Unwrap cg coordinates
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
%%Unwrap kinematic derivatics
xdot = kinematicderivs(1);
ydot = kinematicderivs(2);
zdot = kinematicderivs(3);
phidot = kinematicderivs(4);
thetadot = kinematicderivs(5);
psidot = kinematicderivs(6);

%Trim Velocity
Vtrim = 20;

%%% Longitudinal Coefficients
C_m_0 = 0.0598;
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
C_x_dT =  7.6509;
C_D_u = 0;
C_m_u = 0;

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
C_y_p = 0.0057;
C_y_r = 0.0645;

%%z Geometry and Inertias

% Reference Area (m^2)
S = 0.6558;
% % Wingspan (m)
b = 2.04;
% %Mean chord
c_bar = 0.3215;
%Trim Velocity
Vtrim = 20;


%%% Grab aero forces at aerodynamic center
uaero = u - rcg2ac(2)*r;
vaero = v + r*rcg2ac(1);
waero = w - q*rcg2ac(1) + p*rcg2ac(2);

% Total airspeed
Vaero = norm([uaero vaero waero]);

phat = b * p / (2 * Vaero);
qhat = c_bar * q / (2 * Vaero);
rhat = b * r / (2 * Vaero);

% Dynamic Pressure
rho = 1.220281;
Qa = 1/2*rho*Vaero^2;

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

de = 0;
da = 0;
dT = 0;
dr = 0;

FL = 0;
PID = 0;
METAFL = 1;
if METAFL
  da = controls(1);
  de = controls(3);
  %%%Thrust controller
  UCOMMAND = 20;
  dT = 2.0*(UCOMMAND-uaero);
  %%%Rudder Controller
  dr = 0;
end
if FL
  %%Control Phi
  %%%Use composite Aircraft
  Ixx = I_b(1,1);
  Iyy = I_b(2,2);
  Izz = I_b(3,3);
  %%%Use single Aircraft
  %Ixx = 0.4497;
  %Iyy = 0.5111;
  %Izz = 0.8470;
  C_l_FL = C_l_beta*beta + C_l_p*phat + C_l_r*rhat;
  L_FL = C_l_FL*Qa*S*c_bar;
  fphi = (1/Ixx)*(L_FL+r*q*(Iyy-Izz));
  gphi = (1/Ixx)*(Qa*S*c_bar*C_l_da);
  kd = 0.8;
  kp = 1.2;
  phic = 0*pi/180;
  phidotc = 0;
  phidbldotc = 0;
  gammaphi = phidbldotc - kd*(phidot-phidotc) - kp*(phi-phic);
  da = (gammaphi-fphi)/gphi;
  %%Control Theta
  C_m_FL = C_m_0 + C_m_alpha*alpha + C_m_q*qhat + C_m_u*uaero;
  M_FL = C_m_FL*Qa*S*c_bar;
  ftheta = (1/Iyy)*(M_FL+r*p*(Izz-Ixx));
  gtheta = (1/Iyy)*(Qa*S*c_bar*C_m_de);
  kd = 1.1;
  kp = 1.2;
  %%%Outer loop command of z
  zc = -200;
  thetac = 0.02*(z-zc) + 0.01*zdot;
  if abs(thetac) > 30*pi/180
    thetac = 30*pi/180*sign(thetac);
  end
  thetac = 5*pi/180;
  thetadotc = 0;
  thetadbldotc = 0;
  gammatheta = thetadbldotc - kd*(thetadot-thetadotc) - kp*(theta-thetac);
  de = (gammatheta-ftheta)/gtheta;
  %%%Thrust controller
  UCOMMAND = 20;
  dT = 2.0*(UCOMMAND-uaero);
  %%%Zero out controls
  dr = 0;
  %de = 0;
  %da = 0;
  %dT = 0;
end

UCOMMAND = 20;
ZCOMMAND = -200;
MSLOPE = 0;
BINTERCEPT = 0;
ARCLENGTHPARENT = 10;
if PID
  dT = 2.0*(UCOMMAND-u);
  zerror = z-ZCOMMAND;
  de = -0.3*(0.4*zerror+0.15*zdot);
  lbar0 = cos(MSLOPE);
  lbar1 = sin(MSLOPE);
  planedirection = lbar0*(x-0) + lbar1*(y-BINTERCEPT);
  normAC = sqrt(x*x+y*y);
  flightdirection = sign(xdot*lbar0+ydot*lbar1);
  xs = 0 + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar0;
  ys = BINTERCEPT + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar1;
  rcpath0 = xs - x;
  rcpath1 = ys - y;
  rcpathPsi0 = cos(psi)*rcpath0 + sin(psi)*rcpath1;
  rcpathPsi1 = -sin(psi)*rcpath0 + cos(psi)*rcpath1;
  vpcpathPsi0 = cos(MSLOPE)*xdot + sin(MSLOPE)*ydot;
  vpcpathPsi1 = -sin(MSLOPE)*xdot + cos(MSLOPE)*ydot;
  delpsi = -0.005*rcpathPsi1 + 0.02*vpcpathPsi1;
  PSICOMMANDbody = psi - delpsi;
  phipathcommand = -0.6*delpsi;
  if (abs(phipathcommand) > 45*pi/180)
    phipathcommand = sign(phipathcommand)*45*pi/180;
  end
  da = 1.2*(phi-phipathcommand)+0.1*p;
  KV = 0.25;
  dr = -KV*v;
end

%%%LIMIT CONTROLS%%%%
% if abs(de) > 30*pi/180
%   de = sign(de)*30*pi/180;
% end
% if abs(dr) > 30*pi/180
%   dr = sign(dr)*30*pi/180;
% end
% if abs(da) > 30*pi/180
%   da = sign(da)*30*pi/180;
% end
controls = [da;dr;de;dT];

da = controls(1);
dr = controls(2);
de = controls(3);
dT = controls(4);
controls = [da;dr;de;dT];

%%% Calculation of body forces

C_L = C_L_0 + C_L_alpha*alpha + C_L_q*qhat + C_L_de*de;
C_D = C_D_0 + C_D_alpha*(alpha^2) + C_D_u*uaero;

C_l = C_l_beta*beta + C_l_p*phat + C_l_r*rhat + C_l_da*da + C_l_dr*dr;
C_m = C_m_0 + C_m_alpha*alpha + C_m_q*qhat + C_m_de*de + C_m_u*u;
C_n = C_n_p*phat + C_n_beta*beta + C_n_r*rhat + C_n_da*da + C_n_dr*dr;
C_x = 0;
C_y = C_y_beta*beta + C_y_dr*dr + C_y_p*phat + C_y_r*rhat;
C_z = 0;

Lift = C_L*Qa*S;
Drag = C_D*Qa*S;
T0 = 5.0882;
%thrustscale = T0/5.0882;
thrustscale = 0;
Thrust = T0 + thrustscale*C_x_dT*dT;

Xa = Lift*sin(alpha) - Drag*cos(alpha) + C_x*Qa*S  + Thrust;
Ya = C_y*Qa*S;
Za = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Qa*S;
%%%Include effects of Forces being offset from cg (r cross F)
La = C_l*Qa*S*b + rcg2ac(2)*Za;
Ma = C_m*Qa*S*c_bar - rcg2ac(1)*Za;
Na = C_n*Qa*S*b - rcg2ac(2)*Xa + rcg2ac(1)*Ya;

XYZ = [Xa;Ya;Za];
LMN = [La;Ma;Na];