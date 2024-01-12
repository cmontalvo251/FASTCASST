function [XYZ,LMN] = Point(state,controls)

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
% Wingspan (m)
b = 2.04;
%Mean chord
c_bar = 0.3215;
%Trim Velocity
Vtrim = 20;


%%% Definitions of aero components
uvwwind = [0;0;0];
uvwwindbody = uvwwind;
uaero = u + uvwwindbody(1);
vaero = v + uvwwindbody(2);
waero = w + uvwwindbody(3);

% Total airspeed
Vaero = norm([uaero vaero waero]);

% Dynamic Pressure
rho = 1.22566578494891*pow(1-0.000022569709*abs(z),4.258);
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

da = controls(1);
dr = controls(2);
de = controls(3);
dT = controls(4);
controls = [da;dr;de;dT];

%%% Calculation of body forces

C_L = C_L_0 + C_L_alpha*alpha + C_L_q*q + C_L_de*de;
C_D = C_D_0 + C_D_alpha*(alpha^2) + C_D_u*u;

C_l = C_l_beta*beta + C_l_p*p + C_l_r*r + C_l_da*da + C_l_dr*dr;
C_m = C_m_0 + C_m_alpha*alpha + C_m_q*q + C_m_de*de + C_m_u*u;
C_n = C_n_p*p + C_n_beta*beta + C_n_r*r + C_n_da*da + C_n_dr*dr;
C_x = 0;
C_y = C_y_beta*beta + C_y_dr*dr + C_y_p*p + C_y_r*r;
C_z = 0;

Lift = C_L*Qa*S;
Drag = C_D*Qa*S;
T0 = 5.0882;
Thrust = T0 + C_x_dT*dT;

Xa = Lift*sin(alpha) - Drag*cos(alpha) + C_x*Qa*S  + Thrust;
Ya = C_y*Qa*S;
Za = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Qa*S;
La = C_l*Qa*S*c_bar;
Ma = C_m*Qa*S*c_bar;
Na = C_n*Qa*S*c_bar;

XYZ = [Xa;Ya;Za];
LMN = [La;Ma;Na];