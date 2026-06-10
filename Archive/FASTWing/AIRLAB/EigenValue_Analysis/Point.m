function [Xa,Ya,Za,La,Ma,Na] = Point(state,controls);

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

de = controls(1);
da = controls(2);
dT = controls(3);
dr = controls(4);

% %%% Longitudinal Coefficients

%%Small UAV
C_m_0 = 0.0598;
C_L_0 = 0.062;
C_D_0 = 0.028;
C_L_alpha = 5.195;
C_D_alpha = 1.3537;
C_m_alpha = -0.9317;
C_L_q = 4.589;
C_m_q = -5.263;
C_L_de = 0.2167;
C_m_de = -0.8551;
C_x_dT =  7.6509;
C_T = 0.0317514666500568;
C_T_V = 0;
Ve = 0;
C_D_u = 0;
C_m_u = 0;

%%% Lateral Coefficients

%%Small UAV
C_y_beta = -0.2083;
C_l_beta = -0.0377;
C_n_beta = 0.0116;
C_l_p = -0.4625;
C_n_p = -0.0076;
C_l_r = 0.0288;
C_n_r = -0.0276;
C_y_p = 0.0057;
C_y_r = 0.0645;
C_l_da = -0.2559;
C_n_da = -0.0216;
C_y_dr = 0.1096;
C_l_dr = 0.0085;
C_n_dr = 0.0035;

%Trim Velocity
V = sqrt(u^2 + v^2 + w^2);

%%Non-dimensionalize p,q,r
phat = b*p/(2*V);
qhat = c_bar*q/(2*V);
rhat = b*r/(2*V);

% Dynamic Pressure
rho = 1.220281;
%rho = rhoSI;
Qa = 1/2*rho*V^2;

% Angle of attack:
if abs(u)>0
    alpha = atan2(w,u);
else
    alpha = 0;
end

% Sideslip
if abs(V)>0
    beta = asin(v/V);
else
    beta = 0;
end

%%%CONTROL FORCE%%%

%%% Calculation of body forces

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
qinf = 1/2*rho*V^2*S;
T0 = C_T*qinf;
Thrust = T0 + C_x_dT*dT + C_T_V*(V-Ve);

Xa = Lift*sin(alpha) - Drag*cos(alpha) + C_x*Qa*S  + Thrust;
Ya = C_y*Qa*S;
Za = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Qa*S;
La = C_l*Qa*S*c_bar;
Ma = C_m*Qa*S*c_bar;
Na = C_n*Qa*S*c_bar;
