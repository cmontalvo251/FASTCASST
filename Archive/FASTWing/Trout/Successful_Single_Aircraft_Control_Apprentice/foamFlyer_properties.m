%%% Foam Flyer Aircraft properties
% m = 55.07/grav;
% m = 1.524;      % kg
m = .814;       %grams (1.79 lbs)

%Density of foam is about 0.0229 g/cm^3. or (4 g)/(5.9*19.7*1.5 cm)
%%%% Longitudinal Coefficients
C_L_0 = 0;% .193;
% C_L_alpha = 5.9;
C_L_alpha = 0.537; % From xflyr
C_L_q = 4.589;
C_L_de = 0.2167;
C_D_0 = 0.003;
C_D_alpha = 1.3537;
C_D_u = 0;
C_m_0 = -0.004;         % Change for pitch test
% C_m_alpha = -0.9317;    % Change for pitch test
C_m_alpha = 0.011; % From xflyr
C_m_q = -5.263;         % Change for pitch test
C_m_de = -0.8551;       % Change for pitch test
C_m_u = 0.001;
% C_m_alpha_dot = -2.897;
% C_x_dT =  7.6509;
C_x_dT =  2;


%%% Lateral Coefficients
C_y_beta = -0.2083;
C_y_dr = 0;
C_y_p = 0.0057;
C_y_r = 0;
C_l_beta = -0.0377;
% C_l_p = -0.4625;    % Change for roll test
C_l_p = -.01;%-.82
C_l_r = 0;
% C_l_da = -0.2559;   % Change for roll test
C_l_da = -0.009;%-.01
C_l_dr = 0;
C_n_beta = 0.0116;
C_n_p = -0.0076;
C_n_r = -0;
C_n_da = -0.0216;
C_n_dr = 0;

%%z Geometry and Inertias

% Reference Area (m^2)
% S = 0.6558;
S = 0.19553;
% Wingspan (m)
% b = 2.04;
b = 1.097;
%Mean chord
% c_bar = 0.3215;
c_bar = .1925;
%Trim Velocity  (m/s)
Vtrim = 16.16;
% Thrust
T0 = 0.889644; %(Newtons or 0.2 lb_f)