function [L_T,M_T,N_T,Lift_T,Drag_T,Y_T,G] = ComputeFG(state,rcg2ac,num_ac,Ixx,Iyy)

L_T = 0;
M_T = 0;
N_T = 0;
Lift_T = 0;
Drag_T = 0;
Y_T = 0;
G = zeros(2,2*num_ac);

if ~exist('Ixx','var')
  Ixx = 1;
  Iyy = 1;
end

for ii = 1:num_ac
  [L0i,M0i,N0i,Lifti,Dragi,Y0i,Gi] = ComputeFGi(state,rcg2ac(:,ii));
  L_T = L0i + L_T;
  M_T = M0i + M_T;
  N_T = N0i + N_T;
  Lift_T = Lifti + Lift_T;
  Drag_T = Dragi + Drag_T;
  Y_T = Y0i + Y_T;
  s = 1+(ii-1)*2;
  G(:,s:(s+1)) = [1/Ixx 0;0 1/Iyy]*Gi;
end

function [L0i,M0i,N0i,Lifti,Dragi,Y0i,Gi] = ComputeFGi(state,rcg2ac)

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
    %alpha = asin(waero/Vaero);
else
    alpha = 0;
end

% Sideslip
if abs(Vaero)>0
    beta = asin(vaero/Vaero);
else
    beta = 0;
end

%%%Compute X0i,Y0i,Z0i
C_L = C_L_0 + C_L_alpha*alpha + C_L_q*qhat;
C_D = C_D_0 + C_D_alpha*(alpha^2) + C_D_u*uaero;
C_Y = C_y_beta * beta + C_y_p * phat + C_y_r * rhat;
Drag = C_D*Qa*S;
Lift = C_L*Qa*S;
Lifti = Lift;
Dragi = Drag;
Z0i = -Lift*cos(alpha) - Drag*sin(alpha);
X0i = Lift*sin(alpha) - Drag*cos(alpha);
Y0i = C_Y*Qa*S;

%%Compute L0i
C_l_FL = C_l_beta*beta + C_l_p*phat + C_l_r*rhat;
L0i = C_l_FL*Qa*S*b + rcg2ac(2)*Z0i; 

%%Compute M0i
C_m_FL = C_m_0 + C_m_alpha*alpha + C_m_q*qhat + C_m_u*uaero;
M0i = C_m_FL*Qa*S*c_bar - rcg2ac(1)*Z0i;

%%%Compute N0i
C_n_FL = C_n_p * phat + C_n_beta * beta + C_n_r * rhat;
N0i = C_n_FL*Qa*S*b - rcg2ac(2)*X0i + rcg2ac(1)*Y0i;

%%%Compute G
Gi = zeros(2,2);
Gi(1,1) =  Qa*S*b*C_l_da;
%Gi(1,2) = -Qa*S*rcg2ac(2)*cos(alpha)*C_L_de;
Gi(1,2) = 0;
Gi(2,2) = Qa*S*(c_bar*C_m_de + cos(alpha)*rcg2ac(1)*C_L_de);