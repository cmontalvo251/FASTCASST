purge

syms p

x = 0;y=0;z=0;phi=0;theta=0;psi=0;

u = 8;
v=0;
w=0;
%p=0;
q=0;
r=0;

state = [x,y,z,phi,theta,psi,u,v,w,p,q,r];

da = 0;
dt = 0;
dr = 0;
de = 0;

controls = [da;dt;dr;de];

%%First combined
ub = state(7);
vb = state(8);
wb = state(9);
p = state(10);
q = state(11);
r = state(12);

DA = controls(1);
DELTHRUST = controls(2);
DR = controls(3);
DE = controls(4);

dia = 0.041904;
CHORD = 0.097;
c = CHORD;
Sarea = dia;

C_L_0= 0.00	;
C_D_0= 0.022255 	;
C_m_0= 0.00 	;
C_D_u= 0.00000 	;
C_L_alpha= 4.6132	;
C_D_alpha =1.7426     	;
C_m_alpha= -2.0182 	;
C_m_alpha_dot= 0.00000 	;
C_m_u =0.00000 	;
C_L_q =3.2814 ;
C_m_q =-9.2674	 	;
C_L_de= 0.40623;
C_m_de= -1.1473     	;
C_x_delThrust =5 	;
C_y_beta= -1.2169 	;
C_l_beta= -0.15211  ;
C_n_beta= 0.68532      	;
C_l_p =-0.046095 ;
C_n_p= 0.18438      ;
C_l_r= 0.18438	;
C_n_r= -0.89398	;
C_l_da= -0.50338;
C_n_da= -0.019711	;
C_y_dr= 0.76519	;
C_l_dr= 0.1913      	;
C_n_dr= -0.46376;
C_y_p= 0.0057		;

vel = sqrt(ub^2+vb^2+wb^2);

rho = 1.225;
SOS = 340;

T0 = 0.2882;
T0 = 0;

Qa = 0.5*rho*vel^2;

alpha = atan(wb/ub);
beta = asin(vb/vel);

C_L_alpha_alpha = C_L_alpha*alpha;

C_L = C_L_0 + C_L_alpha_alpha + C_L_q*q + C_L_de*DE;
C_D = C_D_0 + C_D_alpha*alpha^2;
      
C_lmoment = C_l_beta*beta + C_l_p*p + C_l_r*r + C_l_da*DA + C_l_dr*DR;
C_m = C_m_0 + C_m_alpha*alpha + C_m_q*q + C_m_de*DE + C_m_u*ub;
C_n = C_n_p*p + C_n_beta*beta + C_n_r*r + C_n_da*DA + C_n_dr*DR;
C_x = 0;
C_y = C_y_beta*beta + C_y_dr*DR + C_y_p*p;
C_z = 0;

Lift = C_L*Qa*Sarea;
Drag = C_D*Qa*Sarea;

Thrust = T0 + C_x_delThrust*DELTHRUST;
  
XYZ(1) = Lift*sin(alpha) - Drag*cos(alpha) + C_x*Qa*Sarea + Thrust;
XYZ(2) = C_y*Qa*Sarea;
XYZ(3) = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Qa*Sarea
      
LMN(1) = C_lmoment*Qa*Sarea*c;
LMN(2) = C_m*Qa*Sarea*c;
LMN(3) = C_n*Qa*Sarea*c

%now split
ub = state(7);
vb = state(8);
wb = state(9);
pb = state(10);
qb = state(11);
rb = state(12);
rcg = [0;0;0];
den = 1.225;
sos = 340;
vxatm = 0;
vyatm = 0;
vzatm = 0;
dia = 0.041904;
CHORD = 0.097;
ACWING=[    0.0000000000 ; -0.0917000000 ;   0.0000000000];

CLQ=    3.2814; 						    %C_L_q 
CMQ=   -9.2674	; 					    %C_m_q 
CLDE=    0.40623 ;						    %C_L_de 
CMDE=    -1.1473  ;   					    %C_m_de 
CXDT=    5 		;					    %C_x_delThrust 
CYBETA=    -1.2169 		;				    %C_y_beta 
CLBETA=    -0.15211      		;			    %C_l_beta 
CNBETA=    0.68532      			;		    %C_n_beta 
CNP=    0.18438      					;    %C_n_p 
CLR=    0.18438	;					    %C_l_r 
CNR=    -0.89398	;					    %C_n_r 
CLDA=    -0.50338      		;			    %C_l_da 
CNDA=    -0.019711			;			    %C_n_da 
CYDR=    0.76519				;		    %C_y_dr 
CLDR=    0.1913      					;    %C_l_dr 
CNDR=    -0.46376						;    %C_n_dr 
CLP = -0.046095;
CMAC=    -2.0182 ;%						    !C_m_alpha 
CLALPHA = 4.6132;
CL0 = 0;
CD0 = 0.022255;
CDA2 = 1.7426;

AILERON = controls(1);
THRUST = controls(2);
RUDDER = controls(3);
ELEVATOR = controls(4);

% Fuselage calculations

ua = ub;
va = vb;
wa = wb;
vinf = sqrt(ua^2+va^2+wa^2);
alpha = atan(wa/ua);
beta = 0;
beta = asin(va/vinf);
qa = (0.5)*den*vinf^2;
T0 = 0.2882;
T0 = 0;

%!Force on fuselage
FuselageF = [0,0,0];
FuselageF(1) = T0 + CXDT*THRUST;
FuselageF(2) = qa*(CYBETA*beta + CYDR*RUDDER)*dia;
FuselageF(3) = 0;

C_lmoment = CLBETA*beta + CLDA*AILERON + CLDR*RUDDER + CLP/5*pb;
C_m = CMAC*alpha + CMQ*qb + CMDE*ELEVATOR;
C_n = CNBETA*beta + CNDA*AILERON + CNDR*RUDDER + CNR/3*rb;

FuselageM = [0,0,0];
FuselageM(1) = C_lmoment*qa*dia*CHORD;
FuselageM(2) = C_m*qa*dia*CHORD;
FuselageM(3) = C_n*qa*dia*CHORD;

%! Left wing calculations

ua = ub - rb*ACWING(2) + qb*ACWING(3) ;
va = vb + rb*ACWING(1) - pb*ACWING(3);
wa = wb - qb*ACWING(1) + pb*ACWING(2);
vinf = sqrt(ua^2+va^2+wa^2);
alpha = atan(wa/ua);
beta = 0;
beta = asin(va/vinf);
qa = (0.5)*den*vinf^2;

% !//Compute lift on wings
C_L_alpha_alpha = CLALPHA*alpha;

C_L = CL0 + C_L_alpha_alpha + CLQ*qb + CLDE*ELEVATOR;
C_D = CD0 + CDA2*(alpha^2);
      
Lift = C_L*qa*dia/2;
Drag = C_D*qa*dia/2;

salpha = sin(alpha);
calpha = cos(alpha);

LeftWingF = [0,0,0];
LeftWingF(1) = Lift*salpha - Drag*calpha;
LeftWingF(2) = 0;
LeftWingF(3) = -Lift*calpha - Drag*salpha;

LeftWingM = [0,0,0];
LeftWingM(1) = -ACWING(3)*LeftWingF(2) + ACWING(2)*LeftWingF(3);
LeftWingM(2) = ACWING(3)*LeftWingF(1) - ACWING(1)*LeftWingF(3);
LeftWingM(3) = -ACWING(2)*LeftWingF(1) + ACWING(1)*LeftWingF(2);

% ! Right wing calculations
ua = ub + rb*ACWING(2) + qb*ACWING(3);
va = vb + rb*ACWING(1) - pb*ACWING(3);
wa = wb - qb*ACWING(1) - pb*ACWING(2);
vinf = sqrt(ua^2+va^2+wa^2);
alpha = atan(wa/ua);
beta = asin(va/vinf);
qa = (0.5D0)*den*vinf^2;

% !//Compute lift on wings
C_L_alpha_alpha = CLALPHA*alpha;

C_L = CL0 + C_L_alpha_alpha + CLQ*qb + CLDE*ELEVATOR;
C_D = CD0 + CDA2*(alpha^2);
      
Lift = C_L*qa*dia/2;
Drag = C_D*qa*dia/2;

salpha = sin(alpha);
calpha = cos(alpha);

RightWingF = [0,0,0];
RightWingF(1) = Lift*salpha - Drag*calpha;
RightWingF(2) = 0;
RightWingF(3) = -Lift*calpha - Drag*salpha;

RightWingM = [0,0,0];
RightWingM(1) = -ACWING(3)*RightWingF(2) - ACWING(2)*RightWingF(3);
RightWingM(2) = ACWING(3)*RightWingF(1) - ACWING(1)*RightWingF(3);
RightWingM(3) = ACWING(2)*RightWingF(1) + ACWING(1)*RightWingF(2);
 
%!Consolidate Forces
XYZs = FuselageF;
XYZs(1) = FuselageF(1) + LeftWingF(1) + RightWingF(1);
XYZs(2) = FuselageF(2) + LeftWingF(2) + RightWingF(2);
XYZs(3) = FuselageF(3) + LeftWingF(3) + RightWingF(3)

LMNs = XYZs;
LMNs(1) = FuselageM(1) + LeftWingM(1) + RightWingM(1);
LMNs(2) = FuselageM(2) + LeftWingM(2) + RightWingM(2);
LMNs(3) = FuselageM(3) + LeftWingM(3) + RightWingM(3)

