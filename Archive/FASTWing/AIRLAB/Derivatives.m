function [state_dot] = Derivatives(t,state)
%%%%% Equations of motion for a stable fixed wing aircraft.
%%%%%% Unwrap state and control vectors

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

%%% Mass Moment of Inertia Matrix in Body Axis (kg*m^2)
Ix = 0.4497;
Iy = 0.5111;
Iz = 0.8470;
I_b = [Ix 0 0;0 Iy 0;0 0 Iz];
I_b_inv = [1/Ix 0 0;0 1/Iy 0;0 0 1/Iz];
grav = 9.81;
mass = 55.07/grav;

% %%% Longitudinal Coefficients

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

%%%%%%%%%%%PID CONTROLLER!!!!!%%%%%%%%%%%%

%Initialize Controls
de = 0.0;
da = 0;
dT = 0.0;
dr = 0;

%%%PUT YOUR CONTROLLER HERE

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


%%% Calculation of body forces
phat = b*p/(2*Vaero);
qhat = c_bar*q/(2*Vaero);
rhat = b*r/(2*Vaero);

C_L = C_L_0 + C_L_alpha*alpha + C_L_q*qhat + C_L_de*de;
C_D = C_D_0 + C_D_alpha*(alpha^2) + C_D_u*u;

C_l = C_l_beta*beta + C_l_p*phat + C_l_r*rhat + C_l_da*da + C_l_dr*dr;
C_m = C_m_0 + C_m_alpha*alpha + C_m_q*qhat + C_m_de*de + C_m_u*u;
C_n = C_n_p*phat + C_n_beta*beta + C_n_r*rhat + C_n_da*da + C_n_dr*dr;
C_x = 0;
C_y = C_y_beta*beta + C_y_dr*dr + C_y_p*phat + C_y_r*rhat;
C_z = 0;

Lift = C_L*Qa*S;
Drag = C_D*Qa*S;
T0 = 4.8+0.2882;
Thrust = T0 + C_x_dT*dT;

GW = mass*grav;
Xw = -GW*st;
Yw = GW*sp*ct;
Zw = GW*cp*ct;  

Xa = Lift*sin(alpha) - Drag*cos(alpha) + C_x*Qa*S  + Thrust;
Ya = C_y*Qa*S;
Za = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Qa*S;
La = C_l*Qa*S*b;
Ma = C_m*Qa*S*c_bar;
Na = C_n*Qa*S*b;

XYZa = [Xa;Ya;Za];
LMNa = [La;Ma;Na];

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

state_dot = [xyz_dot; pts_dot; uvw_dot; pqr_dot];


