function [XYZ,LMN] = Combined(state,controls)

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

alpha = atan2(wb,ub);
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
XYZ(3) = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Qa*Sarea;
      
LMN(1) = C_lmoment*Qa*Sarea*c;
LMN(2) = C_m*Qa*Sarea*c;
LMN(3) = C_n*Qa*Sarea*c;