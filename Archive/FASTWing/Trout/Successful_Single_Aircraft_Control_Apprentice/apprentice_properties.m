%%% Apprentice R/C Aircraft properties
% m = 55.07/grav;
m = 1.524;      % kg
%%%% Longitudinal Coefficients
C_L_0 = 0.062;
C_L_alpha = 5.195;
C_L_q = 4.589;
C_L_de = 0.2167;
C_D_0 = 0.028;
C_D_alpha = 1.3537;
C_D_u = 0;
C_m_0 = 0.0598;         % Change for pitch test
C_m_alpha = -0.9317;    % Change for pitch test
C_m_q = -5.263;         % Change for pitch test
C_m_de = -0.8551;       % Change for pitch test
C_m_u = 0;
% C_m_alpha_dot = -2.897;
C_x_dT =  7.6509;

%%% Lateral Coefficients
C_y_beta = -0.2083;
C_y_dr = 0.1096;
C_y_p = 0.0057;
C_y_r = 0.0645;
C_l_beta = -0.0377;
% C_l_p = -0.4625;    % Change for roll test
C_l_p = -.8658;%-.82
C_l_r = 0.0288;
% C_l_da = -0.2559;   % Change for roll test
C_l_da = -0.009;%-.01
C_l_dr = 0.0085;
C_n_beta = 0.0116;
C_n_p = -0.0076;
C_n_r = -0.0276;
C_n_da = -0.0216;
C_n_dr = 0.0035;

%%z Geometry and Inertias

% Reference Area (m^2)
% S = 0.6558;
S = 0.336;       % apprentice
% Wingspan (m)
% b = 2.04;
b = 1.5;         % apprentice
%Mean chord
% c_bar = 0.3215;
c_bar = .224;      % apprentice
%Trim Velocity  (m/s)
Vtrim = 20;
% Thrust
T0 = 1.5; %%Based on thrust = drag  %Apprentice