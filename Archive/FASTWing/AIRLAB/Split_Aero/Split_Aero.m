%%%This will split the aero a code

%%%%%INPUTS FROM POINT MODEL AIRCRAFT
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
% Reference Area (m^2)
S = 0.6558;
% Wingspan (m)
b = 2.04;
%Mean chord
c_bar = 0.3215;
%Reference Speed
Vtrim = 20;
%%%Split properties
npanels = 2;
%%%TAIL WING PROPERTIES
bh = 1; %%these must be known otherwise this method wont work
lh = -2; 

%%%%%CALCULATIONS%%%%%

%%%TAIL SIZE
ch = c_bar; %%Just another simplification
SH = bh*ch; %%Size of horizontal tail

%%%MAIN WING
CL0i = C_L_0; %%consequence of symmetric airfoils on tail
CD0i = C_D_0/(1+3*SH/(2*S)); %%assuming only horizontal and main wing
                       %produce drag
CLai = C_L_alpha/(1+SH/S); %%using Xforce eqn while varying w
CDai = C_D_alpha/(1+SH/S); %%using Xforce eqn while varying w
Cmi = C_m_0; %%consequence of symmetric airfoils on tail
lw = 0; %%Simplification
ci = c_bar; %%Defn of panels
Si = S/npanels; %%Defn of panels

%%HORIZONTAL TAIL
CL0h = 0; %%Symmetric airfoil 
CD0h = CD0i; %%Just for simplicity
CLah = CLai; %%Just for simplicity
CDah = CDai; %%Just for simplicity
Cmh = 0; %%Symmetric airfoil

%%VERTICAL TAIL
CL0v = 0; %%Symmetric airfoil
CD0v = CD0h; %%Simplicity
CLav = CLah; %%Simplification
CDav = 0; %%Simplification
Cmv = 0; %%Symmetric airfoil
cv = ch; %%Simplification
Sv = SH/2; %%Assume vertical and horizontal tail are identical with
           %half the length
lv = lh; %%Vertical and horizontal tail should be at the same location
hv = -b/4;

%%FUSELAGE
CDaf = SH*lh^2*CDah/(S*Vtrim^2);
CLqf = C_L_q+(SH/S)*lh*CLah/Vtrim;
Cyrf = Sv*(lv)*CLav/(Vtrim*S);
Cypf = C_y_p - CLav*Sv*hv/(S*Vtrim);
Cnpf = C_n_p - lv*Sv*CLav*hv/(Vtrim*c_bar*S);
Clrf = C_l_r - lv*Sv*CLav*hv/(Vtrim*c_bar*S);
Clbf = C_l_beta-Sv*hv*CLav/(S*c_bar);
Cybf = C_y_beta+(Sv/S)*CLav;
Cnbf = C_n_beta+Sv*lv*CLav/(S*c_bar);
Cmaf = C_m_alpha-SH*lh*CLah/(S*c_bar);
Cmqf = C_m_q+SH*lh^2*CLah/(c_bar*S*Vtrim);
Cnrf = C_n_r+Sv*lv^2*CLav/(c_bar*S*Vtrim);
Clpf = C_l_p+b^2*(CLai-CD0i)/(4*Vtrim*c_bar);
