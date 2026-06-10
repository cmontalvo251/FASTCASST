function [A,B] = Phillips_Controllability_Dimensional()

%%%Compute the controllability of the matrices from Phillips. The
%aircraft is a typical general aviation aircraft page 790

S_T = 185; %ft^2
b_T = 33; %ft^2
c_bar_T = S_T/b_T;
W_T = 2800; %lbf
Vtrim = 180; %ft/s
Ixx = 1000; %slug-ft^2
Iyy = 3000; %slug-ft^2
Izz = 3500; %slug-ft^2
Ixz = 30; %slug-ft^2
Ix1 = Ixx;
Iy1 = Iyy;
Iz1 = Izz;
Ixz = 30; %slug-ft^2
C_D_0_T = 0.05;
C_L_alpha_T = 4.4;
C_D_alpha_T = 0.35;
C_m_alpha_T = -0.68;
C_L_alpha_hat = 1.6;
C_m_alpha_hat = -4.35;
C_y_beta_T = -0.560;
C_l_beta_T = -0.0750;
C_n_beta_T = 0.070;
C_D_q_T = 0;
C_L_q_T = 3.8;
C_m_q_T = -9.95;
C_y_p_T = 0;
C_l_p_T = -0.410;
C_n_p_T = -0.0575;
C_y_r_T = 0.240;
C_l_r_T = 0.105;
C_n_r_T = -0.125;
C_L_de = 0.35; %%%Page 977
C_z_de = -C_L_de;
C_D_de = 0;
C_x_de = -C_D_de;
C_x_dt = 7.65;
C_m_de = -0.920;
C_y_da = 0;
C_l_da = -0.135;
C_n_da = 0.0035;
C_y_dr = 0.155;
C_l_dr = 0.105;
C_n_dr = -0.075;

g = 32.17;
rho = 0.0023769;
theta0 = 0;

%%%To dimensionalize these values we need the Phillips textbook.

%%Longitudinal Approximation
Rgx = g*c_bar_T/(2*Vtrim^2);
R_z_alpha_hat = rho*S_T*c_bar_T/(4*(W_T/g))*(-C_L_alpha_hat);
R_m_alpha_hat = rho*S_T*(c_bar_T^3)/(8*Iyy)*C_m_alpha_hat;
R_x_mu = -rho*S_T*c_bar_T/(4*(W_T/g))*2*C_D_0_T;
theta0 = 0;
C_L_o = W_T*cos(theta0)/(0.5*rho*Vtrim^2*S_T);
R_z_mu = -rho*S_T*c_bar_T/(4*(W_T/g))*2*C_L_o;
R_m_mu = 0;
%%%Compute C_D_alpha properly for stability derivative
%H = [1 0 ;1 10*pi/180];
%X = [0 C_D_alpha_T*(10*pi/180)^2]';
%thetastar = inv(H'*H)*H'*X;
%C_D_alpha_1 = thetastar(2);
R_x_alpha = rho*S_T*c_bar_T/(4*(W_T/g))*(C_L_o-C_D_alpha_T);
R_z_alpha = rho*S_T*c_bar_T/(4*(W_T/g))*(-C_L_alpha_T-C_D_0_T);
R_m_alpha = rho*S_T*c_bar_T^3/(8*Iyy)*C_m_alpha_T;
R_x_q = rho*S_T*c_bar_T/(4*(W_T/g))*-C_D_q_T;
R_z_q = rho*S_T*c_bar_T/(4*(W_T/g))*-C_L_q_T; 
R_m_q = rho*S_T*c_bar_T^3/(8*Iyy)*C_m_q_T;

%%%Long Period Approximation
Rs = R_m_alpha/(R_m_alpha-R_z_alpha*R_m_q);
Rd = R_x_alpha*R_m_q/(R_m_alpha-R_z_alpha*R_m_q);
Rp = Rgx*Rs*(R_z_alpha+R_m_q)/(R_m_alpha-R_z_alpha*R_m_q);

%%%Longintudinal A_long matrix - page 838
A_long = zeros(6,6);
A_long(1,1) = R_x_mu;
A_long(1,2) = R_x_alpha;
A_long(1,3) = R_x_q;
A_long(1,6) = -Rgx*cos(theta0);
A_long(2,1) = R_z_mu;
A_long(2,2) = R_z_alpha;
A_long(2,3) = (1+R_z_q);
A_long(2,6) = -Rgx*sin(theta0);
A_long(3,1) = R_m_mu;
A_long(3,2) = R_m_alpha;
A_long(3,3) = R_m_q;
A_long(3,6) = 0;
A_long(4,1) = cos(theta0);
A_long(4,2) = sin(theta0);
A_long(4,6) = -sin(theta0);
A_long(5,1) = -sin(theta0);
A_long(5,2) = cos(theta0);
A_long(5,6) = -cos(theta0);
A_long(6,3) = 1;

%%%%Longitudinal B_long matrix
B_long = zeros(6,2);
fc = rho*S_T*c_bar_T/(4*W_T/g);
R_x_de = fc*C_x_de;
R_x_dt = fc*C_x_dt;
R_z_de = fc*C_z_de;
R_m_de = rho*S_T*(c_bar_T^3)/(8*Iyy)*C_m_de;
B_long(1,1) = R_x_de;
B_long(2,1) = R_z_de;
B_long(3,1) = R_m_de;
B_long(1,2) = R_x_dt;

%%%Gain matrix 
M_long = eye(6);
M_long(2,2) = 1-R_z_alpha_hat;
M_long(3,2) = -R_m_alpha_hat;

AM_long = inv(M_long)*A_long;
BM_long = inv(M_long)*B_long;

%long_modes_nd = eig(AM_long);

%long_modes = 2*Vtrim/(c_bar_T)*long_modes_nd;

%Wclong = controllability(AM_long,BM_long);
%r = rank(Wclong)

%%%Lateral Values
R_l_p = (rho*S_T*b_T^3/(8*Ixx))*C_l_p_T;
fw = rho*S_T*b_T/(4*W_T/g);
fxx = rho*S_T*b_T^3/(8*Ixx);
fzz = rho*S_T*b_T^3/(8*Izz);
R_y_beta = fw*C_y_beta_T;
R_l_beta = fxx*C_l_beta_T;
R_n_beta = fzz*C_n_beta_T;
R_y_p = fw*C_y_p_T;
R_l_p = fxx*C_l_p_T;
R_n_p = fzz*C_n_p_T;
R_y_r = fw*C_y_r_T;
R_l_r = fxx*C_l_r_T;
R_n_r = fzz*C_n_r_T;
Rgy = g*b_T/(2*Vtrim^2);
R_Ds = (R_l_beta*(Rgy-(1-R_y_r)*R_n_p)-R_y_beta*R_l_r*R_n_p)/R_l_p;
R_Dc = R_l_r*R_n_p/R_l_p;
R_Dp = Rgy*(R_l_r*R_n_beta-R_l_beta*R_n_r)/(R_l_p*(R_n_beta+R_y_beta*R_n_r))-R_Ds/R_l_p;

%%%Lateral A matrix - Page 886
lxz = Ixz/Ixx;
lzx = Ixz/Izz;
A_lat = zeros(6,6);
A_lat(1,1) = R_y_beta;
A_lat(1,2) = R_y_p;
A_lat(1,3) = R_y_r-1;
A_lat(1,5) = Rgy*cos(theta0);
A_lat(2,1) = R_l_beta;
A_lat(2,2) = R_l_p;
A_lat(2,3) = R_l_r;
A_lat(3,1) = R_n_beta;
A_lat(3,2) = R_n_p;
A_lat(3,3) = R_n_r;
A_lat(4,1) = 1;
A_lat(4,6) = cos(theta0);
A_lat(5,2) = 1;
A_lat(5,3) = tan(theta0);
A_lat(6,3) = sec(theta0);

%%%Lateral B Matrix
R_y_da = fw*C_y_da;
R_y_dr = fw*C_y_dr;
R_l_da = fxx*C_l_da;
R_l_dr = fxx*C_l_dr;
R_n_da = fzz*C_n_da;
R_n_dr = fzz*C_n_dr;
B_lat = zeros(6,2);
B_lat(1,1) = R_y_da;
B_lat(1,2) = R_y_dr;
B_lat(2,1) = R_l_da;
B_lat(2,2) = R_l_dr;
B_lat(3,1) = R_n_da;
B_lat(3,2) = R_n_dr;

%%%Gain Matrix
M_lat = eye(6);
M_lat(3,2) = -lzx;
M_lat(2,3) = -lxz;

AM_lat = inv(M_lat)*A_lat;
BM_lat = inv(M_lat)*B_lat;

%lat_modes_nd = eig(AM_lat);

%lat_modes = lat_modes_nd*2*Vtrim/b_T;

%Wclat = controllability(AM_lat,BM_lat);

%r = rank(Wclat);


%%%Combine Lateral and Longitudinal Dynamics
Ap = [A_long,zeros(6,6);zeros(6,6),A_lat];
Bp = [B_long,zeros(6,2);zeros(6,2),B_lat];

%%%Rearrange Matrices such that xlong matches with standard
%convention
order = [4 10 5 11 6 12 1 7 2 8 3 9];
T = zeros(12,12);
for ii = 1:length(order)
  T(ii,order(ii)) = 1;
end

Tu = [1 0 0 0;0 0 1 0;0 0 0 1;0 1 0 0];

A = T*Ap*inv(T);
B = T*Bp*inv(Tu);