function [CL,CD,CM] = AeroModel(uvw);
%%Compute Lift and Drag of a panel given 

u = uvw(1);
%v = uvw(2);
w = uvw(3);

alfa = atan2(w,u);

C_L_0 = 0.062;
C_D_0 = 0.028;
C_M_0 = 0.0598;
%C_L_alpha = 5.195;
C_L_alpha = 3.486;
%C_L_alpha = 3.443;
C_D_alpha = 1.3537;
C_M_alpha = -0.9317;

CL = C_L_0 + C_L_alpha*alfa;
CD = C_D_0 + C_D_alpha*alfa^2;
CM = C_M_0 + C_M_alpha*alfa;