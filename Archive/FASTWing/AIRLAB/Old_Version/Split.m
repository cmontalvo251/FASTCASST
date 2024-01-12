function [XYZ,LMN] = Split(state,controls)

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
ACWING=[    0.0000000000 ; -0.08750000 ;   0.0000000000];

CLQ=    3.2814; 						    %C_L_q 
CMQ=   -9.2674	; 					    %C_m_q 
CLDE=    0.40623 ;						    %C_L_de 
CMDE=    -1.1473  ;   					    %C_m_de 
CXDT=    5 		;					    %C_x_delThrust 
CYBETA=    -1.2169 		;				    %C_y_beta 
CLBETA=    -0.15211      		;			    %C_l_beta 
CNBETA=    0.68532      			;		    %C_n_beta 
CNP=    0.18438      					;    %C_n_p 
CYP = 0.0057;
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
alpha = atan2(wa,ua);
beta = 0;
beta = asin(va/vinf);
qa = (0.5)*den*vinf^2;
T0 = 0.2882;
T0 = 0;

%!Force on fuselage
FuselageF(1) = T0 + CXDT*THRUST;
FuselageF(2) = qa*(CYBETA*beta + CYDR*RUDDER + CYP*pb)*dia;
FuselageF(3) = 0;

C_lmoment = CLBETA*beta + CLDA*AILERON + CLDR*RUDDER + CLR*rb;
C_m = CMAC*alpha + CMQ*qb + CMDE*ELEVATOR;
C_n = CNBETA*beta + CNDA*AILERON + CNDR*RUDDER + CNP*pb + CNR*rb;

FuselageM(1) = C_lmoment*qa*dia*CHORD;
FuselageM(2) = C_m*qa*dia*CHORD;
FuselageM(3) = C_n*qa*dia*CHORD;

%! Left wing calculations

ua = ub - rb*ACWING(2) + qb*ACWING(3);
va = vb + rb*ACWING(1) - pb*ACWING(3);
wa = wb - qb*ACWING(1) + pb*ACWING(2);
vinf = sqrt(ua^2+va^2+wa^2);
alpha = atan2(wa,ua);
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

LeftWingF(1) = Lift*salpha - Drag*calpha;
LeftWingF(2) = 0;
LeftWingF(3) = -Lift*calpha - Drag*salpha;

LeftWingM(1) = -ACWING(3)*LeftWingF(2) + ACWING(2)*LeftWingF(3);
LeftWingM(2) = ACWING(3)*LeftWingF(1) - ACWING(1)*LeftWingF(3);
LeftWingM(3) = -ACWING(2)*LeftWingF(1) + ACWING(1)*LeftWingF(2);

% ! Right wing calculations
uar = ub + rb*ACWING(2) + qb*ACWING(3);
var = vb + rb*ACWING(1) - pb*ACWING(3);
war = wb - qb*ACWING(1) - pb*ACWING(2);
vinfr = sqrt(uar^2+var^2+war^2);
alphar = atan2(war,uar);
betar = asin(var/vinfr);
qar = (0.5D0)*den*vinfr^2;

% !//Compute lift on wings
C_L_alpha_alpha = CLALPHA*alphar;

C_L = CL0 + C_L_alpha_alpha + CLQ*qb + CLDE*ELEVATOR;
C_D = CD0 + CDA2*(alphar^2);
      
Liftr = C_L*qar*dia/2;
Dragr = C_D*qar*dia/2;

salphar = sin(alphar);
calphar = cos(alphar);

RightWingF(1) = Liftr*salphar - Dragr*calphar;
RightWingF(2) = 0;
RightWingF(3) = -Liftr*calphar - Dragr*salphar;

RightWingM(1) = -ACWING(3)*RightWingF(2) - ACWING(2)*RightWingF(3);
RightWingM(2) = ACWING(3)*RightWingF(1) - ACWING(1)*RightWingF(3);
RightWingM(3) = ACWING(2)*RightWingF(1) + ACWING(1)*RightWingF(2);
 
%!Consolidate Forces

XYZ(1) = FuselageF(1) + LeftWingF(1) + RightWingF(1);
XYZ(2) = FuselageF(2) + LeftWingF(2) + RightWingF(2);
XYZ(3) = FuselageF(3) + LeftWingF(3) + RightWingF(3);

LMN(1) = FuselageM(1) + LeftWingM(1) + RightWingM(1);
LMN(2) = FuselageM(2) + LeftWingM(2) + RightWingM(2);
LMN(3) = FuselageM(3) + LeftWingM(3) + RightWingM(3);