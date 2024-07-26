purge

syms u v w rho da dr de dT C_L_0 C_L_alpha C_m_0 C_L_0 C_D_0 C_L_alpha
syms C_D_alpha C_m_alpha C_m_alpha_dot C_L_q C_m_q C_L_de C_m_de 
syms C_x_dT C_D_u C_m_u p q r S c_bar T0 lt lv hv SH CD0v CDav
syms C_y_beta C_l_beta C_n_beta C_l_p C_n_p C_l_r C_n_r C_l_da
syms C_n_da C_y_dr C_l_dr C_n_dr C_y_p b lw CD0t CDat ch cv
syms CL0i CD0i CLai CDai Si ci Cmi CL0t CLat CL0v CLav Sv

%%%%%Things we know
ci = c_bar;
CL0t = 0;
CL0v = 0;
CL0i = C_L_0;
lw = 0;
Cmi = C_m_0;
%CD0v = 0;
%CD0t = 0;
%CDav = 0;
%CDat = 0;
%CD0i = C_D_0;
%CLai = C_L_alpha;
%CDai = C_D_alpha;

v=0;
%w=0;
p=0;
q=0;
r=0;
da=0;
de=0;
dr=0;
dT=0;
T0=0;

%%%%%%%%FIRST DO SINGLE POINT COMPUTATION%%%%%%%%%%%%%

%%% Definitions of aero components

% Total airspeed
syms Vcg
%Vcg = sqrt(u^2+v^2+w^2);

% Dynamic Pressure
Qa = 1/2*rho*Vcg^2;

% Angle of attack:
alpha = atan(w/u);
%salpha = sin(alpha);
%calpha = cos(alpha);
syms salpha calpha

% Sideslip
beta = asin(v/Vcg);

controls = [da;dr;de;dT];

%%% Calculation of body forces

C_L = C_L_0 + C_L_alpha*alpha + C_L_q*q + C_L_de*de;
C_D = C_D_0 + C_D_alpha*(alpha^2) + C_D_u*u;

C_l = C_l_beta*beta + C_l_p*p + C_l_r*r + C_l_da*da + C_l_dr*dr;
C_m = C_m_0 + C_m_alpha*alpha + C_m_q*q + C_m_de*de + C_m_u*u;
C_n = C_n_p*p + C_n_beta*beta + C_n_r*r + C_n_da*da + C_n_dr*dr;
C_x = 0;
C_y = C_y_beta*beta + C_y_dr*dr + C_y_p*p;
C_z = 0;

Lift = C_L*Qa*S;
Drag = C_D*Qa*S;
Thrust = T0 + C_x_dT*dT;

Xa = Lift*salpha - Drag*calpha + C_x*Qa*S  + Thrust;
Ya = C_y*Qa*S;
Za = -Lift*calpha - Drag*salpha + C_z*Qa*S;
La = C_l*Qa*S*c_bar;
Ma = C_m*Qa*S*c_bar;
Na = C_n*Qa*S*c_bar;

XYZ1 = [Xa;Ya;Za]
LMN1 = [La;Ma;Na]

%%%%%%%%%%NOW DO PANEL METHOD%%%%%%%%%

%%%CG STUFF
VcgI = [u;v;w];
PQR = [0 -r q;r 0 -p;-q p 0];

%%%%%%%%%%%FIRST COMPUTE FORCES ON ALL PANELS%%%%%%%%
npanels = 2;
Si = S/npanels;
Fi = [u;u;u];
Mi = Fi;
Fitotal = zeros(3,1);
Mitotal = Fitotal;
for ii = 1:npanels
  ypanel = -b/2 + b*ii/(npanels+1);
  rcgi = [lw;ypanel;0];
  %%Use 2 pts fixed on a rigid body to get aerodynamics at panel
  ViI = VcgI + PQR*rcgi;
  ui = ViI(1);
  vi = ViI(2);
  wi = ViI(3);
  %Vi = sqrt(ui^2+vi^2+wi^2);
  Vi = Vcg;
  alfai  = atan(wi/ui);
  CLi = CL0i + CLai*alfai;
  CDi = CD0i + CDai*alfai^2;
  %salfai = sin(alfai);
  %calfai = cos(alfai);
  syms salfai calfai
  Fi(1,ii) = 1/2*rho*Si*Vi^2*(CLi*salfai-CDi*calfai);
  Fi(2,ii) = 0;
  Fi(3,ii) = 1/2*rho*Si*Vi^2*(-CLi*calfai-CDi*salfai);
  Mi(:,ii) = cross(rcgi,Fi(:,ii))*ci + 1/2*rho*Si*Vi^2*ci*[0;Cmi;0];
  Fitotal = Fitotal + Fi(:,ii);
  Mitotal = Mitotal + Mi(:,ii);
end

%%%%%%%%%%%NOW FUSELAGE COMPONENTS%%%%%%%%%%%%%
alfa = atan(w/u);
beta = atan(v/Vcg);
CLf = C_L_de*de;
CDf = C_D_u*u;
Cyf = C_y_dr*dr;
T = C_x_dT*dT + T0;
Ff = 1/2*rho*S*Vcg^2*[CLf*salpha-CDf*calpha+C_x;Cyf;-CLf*calpha-CDf*salpha+C_z]+[T;0;0];
Mf = 1/2*rho*S*Vcg^2*c_bar*[0;C_m_u*u;0];

%%%%%%%%%%%%NOW COMPUTE H TAIL COMPONENT%%%%%%%%%%%%%%
%%assume alfat = alfa
alfat = alfa;
rcgt = [lt;0;0];
VHI = VcgI + PQR*rcgt;
%VH = sqrt(VHI(1)^2+VHI(2)^2+VHI(3)^2);
VH = Vcg;
CLt = CL0t + CLat*alfat;
CDt = CD0t + CDat*alfat^2;
FH = 1/2*rho*SH*VH^2*[CLt*sin(alfat)-CDt*cos(alfat);0;-CLt*cos(alfat)-CDt*sin(alfat)];
MH = cross(rcgt,FH)*ch;

%%%%%%%%%%%%%NOW COMPUTE V TAIL COMPONENT%%%%%%%%%%%%%
%%assume alfav = beta << 1
alfav = beta;
rcgv = [lv;0;hv];
VvI = VcgI + PQR*rcgv;
%Vv = sqrt(VvI(1)^2+VvI(2)^2+VvI(3)^2);
Vv = Vcg;
CLv = CL0v + CLav*alfav;
CDv = CD0v + CDav*alfav^2;
Fv = 1/2*rho*Sv*Vv^2*[0;-CLav*VvI(2);0];
Mv = cross(rcgv,Fv)*cv;
Mv(2) = 0;

%%%NOW ADD IT ALL UP%%%%

XYZ2 = Fitotal + Ff + FH + Fv
LMN2 = Mitotal + Mf + MH + Mv

simplify(simplify(XYZ1-XYZ2))
simplify(simplify(LMN1-LMN2))