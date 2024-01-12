function AeroForceMom = RigidAerosplit4_carlos(time, state, delev, dailer, drud)

global MASS GRAVITY DENSITY AREA control_out kappa aero_out
global dle dre
% Unwrap State Vector
x = state(1); y = state(2); z = state(3);
phi = state(4); theta = state(5); psi = state(6);
u = state(7); v = state(8); w = state(9);
p = state(10); q = state(11); r = state(12);
DENSITY = 1.225;
%Control Commands
dle = delev + dailer;
dre = delev - dailer;

%Trim Control Commands for zero torque
% dle = -0.1449;
% dre = -0.1449;
% dle = -0.2569;
% dre = -0.2569;
% dle = -0.1537;
% dre = -0.1537;

if abs(dle) > 30*pi/180
    dle = sign(dle)*30*pi/180;
end
if abs(dre) > 30*pi/180
    dre = sign(dre)*30*pi/180;
end
if abs(drud) > 30*pi/180
    drud = sign(drud)*30*pi/180;
end
%control_out(kappa,:) = [time dle dre drud];



%Wing Geometry Data:
e = 0.9899; %efficiency factor
b = 0.80;
c = 0.081666666666667;

Slw = 0.0320;
Srw = 0.0320;
S = 0.0640;
AR = b^2/Slw;

grw = 0;
glw = 0;

%Tail Fuselage Area
Stf = 0.0640;

%Full vehicle non-dimensionalized c.g.:

x_cg_fv = -0.106175045947113;
y_cg_fv = 0;
z_cg_fv = -0.008781645909422;

%Airfoil Data

cd0 = 0.023964477948894; %airfoil drag at zero angle of attack
cd_a = -0.150547230183978; 
cd_a2 = 1.001502778878958; 
Cm_ac = -0.16300;  %pitching moment coeff. about aerodynamic center of airfoil

CL_alpha_rw = 3.323363100687318; %airfoil finite wing lift curve slope
CL_alpha_lw = 3.323363100687318; %airfoil finite wing lift curve slope
CL0 = 0.458866753065263;
CLmax = 1.3005;
aoamax = 0.1936;
CL_a_stall = -1.5928;

alpha_i = 0*pi/180;

x_ac_tf =  -0.107991660722541;
y_ac_tf = 0.0;
z_ac_tf = -0.002044932453260;

cd0ht = 0.001414226796040;
cdaht = 0.001560210801411;
cda2ht = 0.183011260945677;
cl0ht = -0.002169070955252;
claht = 0.530916956601273;
ait = -0.0340906585;
cd0vt = 0.0007353979339406230;
cdavt = 0.0008113096167335152;
cda2vt = 0.095165855691752;
cl0vt = -0.001127916896731;
clavt = 0.176935582618683;
cd0f = 0.003149997912303;
arht = 4.444444444444446;
arvt = 1.730769230769231;

cddle = 0.002804929308707;
cddre = 0.002804929308707;
clfdle = 0.172140385001997;
clfdre = 0.172140385001997;

cyb = -0.176935582618683;
cyp = -0.009123240978776;
cyr = 0.114002731744811;
cydrud = 0.178119419604927;
clb = -0.004561620489388;
clp = -0.003512473720689;
clr = 0.002939132927796;
cldle = 0.009808084829959;
cldre = -0.009808084829959;
cldrud = 0.009354192117347;
cm0 = 0.067683569621252;
cma = -1.888005707913710;
cmq = -11.350541574200582;
cmdle = -0.563350649893937;
cmdre = -0.563350649893937;
cnb = 0.057001365872406;
cnp = 0.002939132927796;
cnr = -0.036726990277836;
cndle = -0.0001558294060392598;
cndre = 0.0001558294060392598;
cndrud = -0.058414527128186;

    x_ac_lw = -0.077463333333333;
    y_ac_lw = -0.183333333333333;
    z_ac_lw = -0.030000000000000;
    x_ac_rw = -0.077463333333333;
    y_ac_rw = 0.183333333333333;
    z_ac_rw = -0.030000000000000;

dxlw = (x_ac_lw - x_cg_fv); dylw = (y_ac_lw - y_cg_fv);  dzlw = (z_ac_lw - z_cg_fv);
rlw = [dxlw; dylw; dzlw];
dxrw = (x_ac_rw - x_cg_fv); dyrw = (y_ac_rw - y_cg_fv); dzrw = (z_ac_rw - z_cg_fv);
rrw = [dxrw; dyrw; dzrw];
dxtf = (x_ac_tf - x_cg_fv); dytf = (y_ac_tf - y_cg_fv); dztf = (z_ac_tf - z_cg_fv);
rtf = [dxtf; dytf; dztf];


    omega = [p; q; r];
    v_rw = [u;v;w] + cross(omega,rrw);
    vrw = [1 0 0; 0 cos(grw) sin(grw); 0 -sin(grw) cos(grw)]*v_rw;
    arw = atan2(vrw(3),vrw(1)) + alpha_i;%vrw(3)/u;
    if arw > aoamax
        Clift_rw = CLmax + CL_a_stall*(arw-aoamax);
    else
        Clift_rw = CL0 + CL_alpha_rw*arw;
    end
    Vrw = sqrt((vrw(1))^2+(vrw(2))^2+(vrw(3))^2);
    Lrw = 1/2*DENSITY*Vrw^2*Srw*(Clift_rw);
    Drw = 1/2*DENSITY*Vrw^2*Srw*(cd0 + cd_a*arw + cd_a2*arw^2+ (1/(pi*e*AR))*(Clift_rw)^2);
    Frw = [1 0 0; 0 cos(grw) -sin(grw); 0 sin(grw) cos(grw)]*[Lrw*sin(arw) - Drw*cos(arw); 0; -Lrw*cos(arw)-Drw*sin(arw)];
    Mrw = cross(rrw,Frw)+[1 0 0; 0 cos(grw) -sin(grw); 0 sin(grw) cos(grw)]*[0;1/2*DENSITY*Vrw^2*Srw*c*Cm_ac;0];

    v_lw = [u;v;w] + cross(omega,rlw);
    vlw = [1 0 0; 0 cos(glw) sin(glw); 0 -sin(glw) cos(glw)]*v_lw;
    alw = atan2(vlw(3),vlw(1)) + alpha_i; % vlw(3)/u;
    if alw > aoamax
        Clift_lw = CLmax + CL_a_stall*(alw-aoamax);
    else
        Clift_lw = CL0 + CL_alpha_lw*alw;
    end
    Vlw = sqrt((vlw(1))^2 + (vlw(2))^2 + (vlw(3))^2);
    Llw = 1/2*DENSITY*Vlw^2*Slw*(Clift_lw);
    Dlw = 1/2*DENSITY*Vlw^2*Slw*(cd0 + cd_a*alw + cd_a2*alw^2 + 1/(pi*e*AR)*(Clift_lw)^2);
    Flw = [1 0 0; 0 cos(glw) -sin(glw); 0 sin(glw) cos(glw)]*[Llw*sin(alw)-Dlw*cos(alw); 0; -Llw*cos(alw)-Dlw*sin(alw)];
    Mlw = cross(rlw, Flw)+[1 0 0; 0 cos(glw) -sin(glw); 0 sin(glw) cos(glw)]*[0;1/2*DENSITY*Vlw^2*Slw*c*Cm_ac;0];

%     gtf = -0.400351734918030; % k = 45, T = 18
% gtf = -0.050021779031866; %k = 200, T = 10
gtf = 0;
Ttf = [1 0 0; 0 cos(-gtf) sin(-gtf); 0 -sin(-gtf) cos(-gtf)]; %Tranformation from tf frame to body frame
vtf = [u;v;w] + cross(omega,rtf);
vtf_tf = Ttf'*vtf;
% vtf_tf = vtf;
Vtf = sqrt((vtf_tf(1))^2 + (vtf_tf(2))^2 + (vtf_tf(3))^2);
atf = atan2(vtf_tf(3),vtf_tf(1)); %vtf(3)/u;
btf = asin(vtf_tf(2)/Vtf); %vtf(2)/u;
cdht = cd0ht + cdaht*(atf+ait) + cda2ht*(atf+ait)^2 + (cl0ht+claht*(atf+ait))^2/(pi*e*arht) + cddle*dle + cddre*dre;
cdvt =  cd0vt + cdavt*btf + cda2vt*btf^2 + (cl0vt + clavt*btf)^2/(pi*e*arvt);
cliftht = cl0ht + claht*(atf+ait) + clfdle*dle+clfdre*dre;
cliftvt = cl0vt + clavt*btf;
cx = -cd0f*cos(atf)-cdht*cos(atf+ait)-cdvt*cos(btf) + cliftht*sin(atf+ait) + cliftvt*sin(btf);
cz = -cd0f*sin(atf)-cdht*sin(atf+ait) - cliftht*cos(atf+ait);
cy = cyp*(p*b/(2*Vtf)) + cyr*(r*b/(2*Vtf)) + cyb*btf + cydrud*drud;
X_tf = 1/2*DENSITY*Vtf^2*Stf*cx;
Y_tf = 1/2*DENSITY*Vtf^2*Stf*cy;
Z_tf = 1/2*DENSITY*Vtf^2*Stf*cz;
L_tf = 1/2*DENSITY*Vtf^2*Stf*b*(clb*btf + clp*(p*b/(2*Vtf)) + clr*(r*b/(2*Vtf)) + cldle*dle + cldre*dre + cldrud*drud);
M_tf = 1/2*DENSITY*Vtf^2*Stf*c*(cm0 + cma*atf + cmq*(q*c/(2*Vtf)) + cmdle*dle + cmdre*dre);
N_tf = 1/2*DENSITY*Vtf^2*Stf*b*(cnb*btf + cnp*(p*b/(2*Vtf)) + cnr*(r*b/(2*Vtf)) + cndle*dle + cndre*dre + cndrud*drud);


Ftf = [X_tf;Y_tf;Z_tf];
Ftf = Ttf*Ftf;
Mtf = Ttf*[L_tf;M_tf;N_tf] + cross(rtf,Ftf);
% Ftf = zeros(3,1); Mtf = zeros(3,1);
% Frw = zeros(3,1); Mrw = zeros(3,1);
% Flw = zeros(3,1); Mlw = zeros(3,1);
F = Frw + Flw + Ftf;
M = Mrw + Mlw + Mtf;

% aero_out(kappa,:) = [time F(1) F(2) F(3) M(1) M(2) M(3)];
% if kappa == 1
%     aero_out(kappa,:) = [time Frw(1) Frw(2) Frw(3) Mrw(1) Mrw(2) Mrw(3) Flw(1) Flw(2) Flw(3) Mlw(1) Mlw(2) Mlw(3) Ftf(1) Ftf(2) Ftf(3) Mtf(1) Mtf(2) Mtf(3)];
% elseif mod(kappa,200) == 0
%     aero_out(kappa,:) = [time Frw(1) Frw(2) Frw(3) Mrw(1) Mrw(2) Mrw(3) Flw(1) Flw(2) Flw(3) Mlw(1) Mlw(2) Mlw(3) Ftf(1) Ftf(2) Ftf(3) Mtf(1) Mtf(2) Mtf(3)];
% end
%  kappa = kappa + 1;
AeroForceMom = [F;M];