function [Xa,Ya,Za,La,Ma,Na] = Panels(state,controls)

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

% % Reference Area (m^2)
% S = 0.6558;
% % Wingspan (m)
% b = 2.04;
% %Mean chord
% c_bar = 0.3215;

% % Reference Area (m^2)
S = 0.0640;
% % Wingspan (m)
b = 0.80;
% %Mean chord
c_bar = 0.081667;
%efficiency factor
e = 0.9899; 

da = controls(1);
dr = controls(2);
de = controls(3);
dT = controls(4);
controls = [da;dr;de;dT];

%%%%%%%%POINT COEFFICIENTS%%%%

%%Small UAV
% C_m_0 = 0.0598;
% C_L_0 = 0.062;
% C_D_0 = 0.028;
% C_L_q = 4.589;
% C_m_q = -5.263;
% C_L_alpha = 5.195;
% C_D_alpha = 1.3537;
% C_m_alpha = -0.9317;
C_L_de = 0.2167;
C_m_de = -0.8551;
C_x_dT =  7.6509;
C_x = 0;
C_z = 0;
C_D_u = 0;
C_m_u = 0;
C_l_da = -0.2559;
C_n_da = -0.0216;
C_y_dr = 0.1096;
C_l_dr = 0.0085;
C_n_dr = 0.0035;
% C_y_p = 0.0057;
% C_y_beta = -0.2083;
% C_l_beta = -0.0377;
% C_n_beta = 0.0116;
% C_l_p = -0.4625;
% C_n_p = -0.0076;
% C_l_r = 0.0288;
% C_n_r = -0.0276;
% C_l_da = -0.2559;
% C_n_da = -0.0216;
% C_y_dr = 0.1096;
% C_l_dr = 0.0085;
% C_n_dr = 0.0035;
% C_y_p = 0.0057;
% Vtrim = 20;

%%%%%SIZE OF AIRCRAFT
npanels = 2;
Si = S/npanels;
ci = c_bar;
ch = c_bar;
cv = ch;
bt = 1.0;
SH = bt*ch;
Sv = SH/2;

%%%%%%%%%%PANEL COEFFICIENTS
%%Main Wing
% CL0i = C_L_0;
% CD0i = C_D_0/(1+3*SH/(2*S));
% CLai = C_L_alpha/(1+SH/S);
% CDai = C_D_alpha/(1+SH/S);
% Cmi = C_m_0;
% %%%Horizontal tail
% CL0t = 0;
% CD0t = CD0i;
% CLat = CLai;
% CDat = CDai;
% lt = -sqrt(-S*c_bar*C_m_q*Vtrim/(SH*CLat)); 
% %%If lt is nonsensical recompute lt based on your own measurement
% lt = -2;
% %%Vertical Tail
% CL0v = 0;
% CD0v = CD0t;
% CDav = 0;
% CLav = -S*C_y_beta/Sv;
% lv = -sqrt((-S*c_bar*C_n_r*Vtrim)/(Sv*CLav));
% hv = Vtrim*S*c_bar*C_l_r/(Sv*lv*CLav);
% %%If hv or lv is nonsensical recompute lt based on your own measurement
% lv = lt;
% hv = -b/4;
% %%Sometime CLav is nonsensical as well. In that case
% CLav = CLat;
% %%Fuselage
% CDaf = SH*lt^2*CDat/(S*Vtrim^2);
% CLqf = C_L_q+(SH/S)*lt*CLat/Vtrim;
% Cyrf = Sv*(lv)*CLav/(Vtrim*S); %1.2
% %Cypf = C_y_p - CLav*Sv*hv/(S*Vtrim);
% Cypf = C_y_p;
% %Cnpf = C_n_p - lv*Sv*CLav*hv/(Vtrim*c_bar*S);
% %w0 = w;
% %Cnpf = C_n_p - lv*Sv*CLav*hv/(Vtrim*c_bar*S) + ((b/2)^2)*(CL0i*Vtrim-2*(CDai-CLai)*w0)/(c_bar*Vtrim^2);
% Cnpf = C_n_p - lv*Sv*CLav*hv/(Vtrim*c_bar*S);
% Cnpfa = ((b/(2*npanels))^2)*(-2*(CDai-CLai))/(c_bar*Vtrim);
% Clrf = C_l_r - lv*Sv*CLav*hv/(Vtrim*c_bar*S);
% Clbf = C_l_beta-Sv*hv*CLav/(S*c_bar);
% Cybf = C_y_beta+(Sv/S)*CLav;
% Cnbf = C_n_beta+Sv*lv*CLav/(S*c_bar);
% Cmaf = C_m_alpha-SH*lt*CLat/(S*c_bar);
% Cmqf = C_m_q+SH*lt^2*CLat/(c_bar*S*Vtrim);
% %Cnrf = C_n_r+Sv*lv^2*CLav/(c_bar*S*Vtrim);
% Cnrf = C_n_r;
% %Cnrf = 0.43040;
% Clpf = C_l_p+(b/(2*npanels))^2*(CLai-CD0i)/(Vtrim*c_bar);

%%Emilys UAV
x_cg_fv = -0.106175045947113;
y_cg_fv = 0;
z_cg_fv = -0.008781645909422;
x_ac_tf =  -0.107991660722541;
y_ac_tf = 0.0;
z_ac_tf = -0.002044932453260;
dxtf = (x_ac_tf - x_cg_fv); dytf = (y_ac_tf - y_cg_fv); dztf = (z_ac_tf - z_cg_fv);
rtf = [dxtf; dytf; -dztf];
x_ac_lw = -0.077463333333333;
y_ac_lw = -0.183333333333333;
z_ac_lw = -0.030000000000000;
x_ac_rw = -0.077463333333333;
y_ac_rw = 0.183333333333333;
z_ac_rw = -0.030000000000000;
dxlw = (x_ac_lw - x_cg_fv); dylw = (y_ac_lw - y_cg_fv);  dzlw = (z_ac_lw - z_cg_fv);
rlw = [dxlw; dylw; dzlw];
lw = rlw(1);

CL0i = 0.4588;
CD0i = 0.02396;
CLai = 3.3433;
CDai = 1.0015;
Cmi = -0.163;
%%%Horizontal tail
CL0t = -0.00216;
CD0t = 0.0014;
CLat = 0.5309;
ait = -0.03409;
CDat = 0.183;
lt = rtf(1);
arht = 4.444444444444446;
%%Vertical Tail
CL0v = -0.0011; 
CD0v = 0.000735;
CDav = 0.09516;
CLav = 0.1769;
lv = rtf(1);
hv = rtf(3);
arvt = 1.730769230769231;
%%Fuselage
CD0f = 0.0031499;
CDaf = 0;
CLqf = 0;
Cyrf = 0.114002731744811;
Cypf = -0.009123240978776;
Cnpf = 0.002939132927796;
Cnpfa = 0;
Clrf =  0.002939132927796;
Clbf = -0.004561620489388;
Cybf = -0.176935582618683;
Cnbf = 0.057001365872406;
Cmaf = -1.888005707913710;
Cmqf = -11.350541574200582;
Cnrf = -0.036726990277836;
Clpf = -0.003512473720689;

%%%CG STUFF
VcgI = [u;v;w];
PQR = [0 -r q;r 0 -p;-q p 0];
rho = 1.22021;

%%%%%%%%%%%FIRST COMPUTE FORCES ON ALL PANELS%%%%%%%%
Fi = zeros(3,npanels);
Mi = Fi;
Fitotal = zeros(3,1);
Mitotal = Fitotal;
%figure()
%hold on
for ii = 1:npanels
  repeater = round(ii/2);
  %ypanel = (-0.95*b/2 + (repeater-1)*(b)/(npanels+1))*(-1)^ii;
  ypanel = -b/2 + (b/(2*npanels)) + (ii-1)*b/npanels;
  rcgi = [lw;ypanel;0];
  %plot3(rcgi(1),rcgi(2),-rcgi(3),'bx','MarkerSize',10);
  %%Use 2 pts fixed on a rigid body to get aerodynamics at panel
  ViI = VcgI + PQR*rcgi;
  ui = ViI(1);
  vi = ViI(2);
  wi = ViI(3);
  Vi = sqrt(ui^2+vi^2+wi^2);
  alfai  = atan2(wi,ui);
  CLi = CL0i + CLai*alfai;
  AR = (b^2)/Si;
  CDi = CD0i + CDai*alfai^2 + (1/(pi*e*AR))*(CLi)^2;
  salfa = sin(alfai);
  calfa = cos(alfai);
  Fi(:,ii) = 1/2*rho*Si*Vi^2*[CLi*salfa-CDi*calfa;0;-CLi*calfa-CDi*salfa];
  Mi(:,ii) = cross(rcgi,Fi(:,ii))+1/2*rho*Si*Vi^2*ci*[0;Cmi;0];
  Fitotal = Fitotal + Fi(:,ii);
  Mitotal = Mitotal + Mi(:,ii);
end


%%%%%%%%%%%%NOW COMPUTE H TAIL COMPONENT%%%%%%%%%%%%%%
rcgt = [lt;0;0];
VHI = VcgI + PQR*rcgt;
VH = norm(VHI);
alfat = atan2(VHI(3),VHI(1));
betat = asin(VHI(2)/VH);
CLt = CL0t + CLat*(alfat+ait);
CDt = CD0t + CDat*(alfat+ait)^2 + (CL0t + CLat*(alfat+ait))^2/(pi*e*arht);
salfa = sin(alfat);
calfa = cos(alfat);
FH = 1/2*rho*SH*VH^2*[CLt*salfa-CDt*calfa;0;-CLt];
MH = cross(rcgt,FH);

%%%%%%%%%%%%%NOW COMPUTE V TAIL COMPONENT%%%%%%%%%%%%%
rcgv = [lv;0;hv];
VvI = VcgI + PQR*rcgv;
Vv = norm(VvI);
alfav = asin(VvI(2)/Vv);
CLv = CL0v + CLav*alfav;
CDv = CD0v + CDav*alfav^2 + (CL0v + CLav*alfav)^2/(pi*e*arvt);
Fv = 1/2*rho*Sv*Vv^2*[-CDv;-CLv;0];
Mv = cross(rcgv,Fv);
Mv(2) = 0;

%%%%%%%%%%%NOW FUSELAGE COMPONENTS%%%%%%%%%%%%%
Vcg = sqrt(u^2+v^2+w^2);
alfa = atan2(w,u);
beta = asin(v/Vcg);
CLf = C_L_de*de+CLqf*q;
CDf = C_D_u*u;
Cyf = Cypf*(p*b/(2*Vcg))
Cxf = -CD0f*cos(alfa)-CDt*cos(alfat+ait)-CDv*cos(alfav) + CLt*sin(alfat+ait) ...
      + CLv*sin(alfav);
Czf = -CD0f*sin(alfa)-CDt*sin(alfat+ait)-CLt*cos(alfat+ait);
Cmf = Cmqf*(q*c_bar)/(2*Vcg)+Cmaf*alfa+C_m_u*u+C_m_de*de;
Cnf = C_n_da*da + C_n_dr*dr + Cnrf*r;
Clf = C_l_da*da + C_l_dr*dr + Clpf*p;
T0 = 5.0882;
T = C_x_dT*dT + T0;
salfa = sin(alfa);
calfa = cos(alfa);
Ff = 1/2*rho*S*Vcg^2*[CLf*salfa-CDf*calfa+Cxf;Cyf;-CLf*calfa-CDf*salfa+C_z]+[T;0;0];
Mf = 1/2*rho*S*Vcg^2*c_bar*[Clf;Cmf;Cnf];

%%%NOW ADD IT ALL UP%%%%

XYZ = Fitotal + Ff + FH + Fv;
LMN = Mitotal + Mf + MH + Mv;
Bkdn = [Fitotal;Ff;FH;Fv;Mitotal;Mf;MH;Mv];

Xa = XYZ(1);
Ya = XYZ(2);
Za = XYZ(3);
La = LMN(1);
Ma = LMN(2);
Na = LMN(3);

%pause