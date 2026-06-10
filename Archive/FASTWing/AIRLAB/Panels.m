function [XYZ,LMN,Bkdn] = Panels(state,controls)

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

% Reference Area (m^2)
S = 0.6558;
% Wingspan (m)
b = 2.04;
%Mean chord
c_bar = 0.3215;

da = controls(1);
dr = controls(2);
de = controls(3);
dT = controls(4);
controls = [da;dr;de;dT];

%%%%%%%%POINT COEFFICIENTS%%%%

C_m_0 = 0.0598;
C_L_0 = 0.062;
C_D_0 = 0.028;
C_L_q = 4.589;
C_m_q = -5.263;
C_L_alpha = 5.195;
C_D_alpha = 1.3537;
C_m_alpha = -0.9317;
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
C_y_p = 0.0057;
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
Vtrim = 20;

%%%%%SIZE OF AIRCRAFT
npanels = 3;
Si = S/npanels;
ci = c_bar;
ch = c_bar;
cv = ch;
bt = 1.0;
SH = bt*ch;
Sv = SH/2;
lw = 0;

%%%%%%%%%%PANEL COEFFICIENTS
%%Main Wing
CL0i = C_L_0;
CD0i = C_D_0/(1+3*SH/(2*S));
CLai = C_L_alpha/(1+SH/S);
CDai = C_D_alpha/(1+SH/S);
Cmi = C_m_0;
%%%Horizontal tail
CL0t = 0;
CD0t = CD0i;
CLat = CLai;
CDat = CDai;
lt = -sqrt(-S*c_bar*C_m_q*Vtrim/(SH*CLat)); 
%%If lt is nonsensical recompute lt based on your own measurement
lt = -2;
%%Vertical Tail
CL0v = 0;
CD0v = CD0t;
CDav = 0;
CLav = -S*C_y_beta/Sv;
lv = -sqrt((-S*c_bar*C_n_r*Vtrim)/(Sv*CLav));
hv = Vtrim*S*c_bar*C_l_r/(Sv*lv*CLav);
%%If hv or lv is nonsensical recompute lt based on your own measurement
lv = lt;
hv = -b/4;
%%Sometime CLav is nonsensical as well. In that case
CLav = CLat;
%%Fuselage
CDaf = SH*lt^2*CDat/(S*Vtrim^2);
CLqf = C_L_q+(SH/S)*lt*CLat/Vtrim;
Cyrf = Sv*(lv)*CLav/(Vtrim*S); %1.2
%Cypf = C_y_p - CLav*Sv*hv/(S*Vtrim);
Cypf = C_y_p;
%Cnpf = C_n_p - lv*Sv*CLav*hv/(Vtrim*c_bar*S);
%w0 = w;
%Cnpf = C_n_p - lv*Sv*CLav*hv/(Vtrim*c_bar*S) + ((b/2)^2)*(CL0i*Vtrim-2*(CDai-CLai)*w0)/(c_bar*Vtrim^2);
Cnpf = C_n_p - lv*Sv*CLav*hv/(Vtrim*c_bar*S);
Cnpfa = ((b/(2*npanels))^2)*(-2*(CDai-CLai))/(c_bar*Vtrim);
Clrf = C_l_r - lv*Sv*CLav*hv/(Vtrim*c_bar*S);
Clbf = C_l_beta-Sv*hv*CLav/(S*c_bar);
Cybf = C_y_beta+(Sv/S)*CLav;
Cnbf = C_n_beta+Sv*lv*CLav/(S*c_bar);
Cmaf = C_m_alpha-SH*lt*CLat/(S*c_bar);
Cmqf = C_m_q+SH*lt^2*CLat/(c_bar*S*Vtrim);
%Cnrf = C_n_r+Sv*lv^2*CLav/(c_bar*S*Vtrim);
Cnrf = C_n_r;
%Cnrf = 0.43040;
Clpf = C_l_p+(b/(2*npanels))^2*(CLai-CD0i)/(Vtrim*c_bar);

%%%CG STUFF
VcgI = [u;v;w];
PQR = [0 -r q;r 0 -p;-q p 0];
rho = 1.22566578494891*pow(1-0.000022569709*abs(z),4.258);

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
  CDi = CD0i + CDai*alfai^2;
  salfa = sin(alfai);
  calfa = cos(alfai);
  Fi(:,ii) = 1/2*rho*Si*Vi^2*[CLi*salfa-CDi*calfa;0;-CLi*calfa-CDi*salfa];
  Mi(:,ii) = cross(rcgi,Fi(:,ii))+1/2*rho*Si*Vi^2*ci*[0;Cmi;0];
  Fitotal = Fitotal + Fi(:,ii);
  Mitotal = Mitotal + Mi(:,ii);
end

%%%%%%%%%%%NOW FUSELAGE COMPONENTS%%%%%%%%%%%%%
Vcg = sqrt(u^2+v^2+w^2);
alfa = atan2(w,u);
beta = asin(v/Vcg);
CLf = C_L_de*de+CLqf*q;
CDf = C_D_u*u;
%Cyf = C_y_dr*dr+Cyrf*r + Cypf*p + Cybf*beta;
Cyf = C_y_dr*dr + Cypf*p;
%Cxf = C_x + CDaf*q^2;
Cxf = C_x;
Cmf = Cmqf*q+Cmaf*alfa+C_m_u*u+C_m_de*de;
%Cnf = Cnrf*r+Cnbf*beta+Cnpf*p+Cnpfa*alfa*p + C_n_da*da+C_n_dr*dr;
Cnf = C_n_da*da + C_n_dr*dr + Cnrf*r;
%Clf = Clbf*beta + Clpf*p + Clrf*r + C_l_da*da + C_l_dr*dr;
Clf = C_l_da*da + C_l_dr*dr + Clpf*p;
T0 = 5.0882;
T = C_x_dT*dT + T0;
salfa = sin(alfa);
calfa = cos(alfa);
Ff = 1/2*rho*S*Vcg^2*[CLf*salfa-CDf*calfa+Cxf;Cyf;-CLf*calfa-CDf*salfa+C_z]+[T;0;0];
Mf = 1/2*rho*S*Vcg^2*c_bar*[Clf;Cmf;Cnf];

%%%%%%%%%%%%NOW COMPUTE H TAIL COMPONENT%%%%%%%%%%%%%%
rcgt = [lt;0;0];
%plot3(rcgt(1),rcgt(2),-rcgt(3),'rx','MarkerSize',10)
VHI = VcgI + PQR*rcgt;
VH = norm(VHI);
alfat = atan2(VHI(3),VHI(1));
CLt = CL0t + CLat*alfat;
CDt = CD0t + CDat*alfat^2;
salfa = sin(alfat);
calfa = cos(alfat);
FH = 1/2*rho*SH*VH^2*[CLt*salfa-CDt*calfa;0;-CLt];
%FH = 1/2*rho*SH*VH^2*[-CDt;0;-CLt];
MH = cross(rcgt,FH);

%%%%%%%%%%%%%NOW COMPUTE V TAIL COMPONENT%%%%%%%%%%%%%
hv = 0.11*hv;
lv = 0.01*lv;
rcgv = [lv;0;hv];
%plot3(rcgv(1),rcgv(2),-rcgv(3),'gx','MarkerSize',10)
VvI = VcgI + PQR*rcgv;
Vv = norm(VvI);
alfav = asin(VvI(2)/Vv);
CLav = 0.25*CLav;
CLv = CL0v + CLav*alfav;
CDv = CD0v + CDav*alfav^2;
Fv = 1/2*rho*Sv*Vv^2*[-CDv;-CLv;0];

Mv = cross(rcgv,Fv);
Mv(2) = 0;

%%%NOW ADD IT ALL UP%%%%

XYZ = Fitotal + Ff + FH + Fv;
LMN = Mitotal + Mf + MH + Mv;
Bkdn = [Fitotal;Ff;FH;Fv;Mitotal;Mf;MH;Mv];

%pause