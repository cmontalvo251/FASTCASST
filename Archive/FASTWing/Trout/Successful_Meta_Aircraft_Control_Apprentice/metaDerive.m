function dxdt = metaDerive(xstate,t,controller,n,Ts,Fs)
global da de Fymag dely
%% Aircraft Properties %%
%run airplane_calcs%%% Aircraft properties
% foamFlyer_properties
apprentice_properties

%% Kinematics%%
xyz = xstate(1:3,n);
uvw = xstate(7:9,n);

%%%First Aircraft
phi = xstate(4,n);
theta = xstate(5,n);
psi = xstate(6,n);
ptp = [phi;theta;psi];
T3 = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];
T2 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
T1 = [1 0 0;0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
T321 = T3*T2*T1;

xyzdot = T321*uvw;

%%%Rot Kinematics
pqr = xstate(10:12,n);
H = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);0 cos(phi) -sin(phi);0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
ptpdot = H*pqr;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%THIS IS A FUNCTION OF YOUR AIRCRAFT

%%%%MASS PROPS HERE
%%% Mass Moment of Inertia Matrix in Body Axis (kg.m^2)
% Ixx = 0.01; %kg*m^2 (from xflyer sim)
% Iyy = 0.08;
% Izz = 0.009;

MOI = MassMOI;      % For apprentice
Ixx = MOI(1);
Iyy = MOI(2);
Izz = MOI(3);

I = [Ixx 0 0;0 Iyy 0;0 0 Izz];
Iinv = [1/Ixx 0 0;0 1/Iyy 0;0 0 1/Izz];

% Total airspeed
u = uvw(1);
v = uvw(2);
w = uvw(3);
uaero = u;
vaero = v;
waero = w;
Vaero = norm([uaero vaero waero]);

% Dynamic Pressure
% rho = 1.22566578494891;
rho = 1.220281; %200m
Qa = 1/2*rho*Vaero^2;

% Angle of attack:
if abs(u)>0
     alpha = atan2(waero,uaero);
else
     alpha = 0;
end

beta = asin(v/Vaero);

%% CONTROL LOOP %%
if controller == 1 %PID
    [de,da,dT,dr] =  PIDcontroller(xyz,ptp,uvw,pqr,xyzdot,ptpdot,t);
else
    de = 0;
    da = 0;
    dT = 0;
    dr = 0;
end

%%%LIMIT CONTROLS%%%%
if abs(de) > 45*pi/180
  de = sign(de)*45*pi/180;
end
if abs(dr) > 45*pi/180
  dr = sign(dr)*45*pi/180;
end
if abs(da) > 45*pi/180
  da = sign(da)*45*pi/180;
end

%% Calculation of body forces %%

p = pqr(1);
q = pqr(2);
r = pqr(3);
phat = b*p/(2*Vaero);
qhat = c_bar*q/(2*Vaero);
rhat = b*r/(2*Vaero);
%--------------------------------------------------------------------%
C_L = C_L_0 + C_L_alpha*alpha + C_L_q*qhat + C_L_de*de;
C_D = C_D_0 + C_D_alpha*(alpha^2) + C_D_u*u;

C_l = C_l_beta*beta + C_l_p*phat + C_l_r*rhat + C_l_da*da + C_l_dr*dr;
C_m = C_m_0 + C_m_alpha*alpha + C_m_q*qhat + C_m_de*de + C_m_u*u;
C_n = C_n_p*phat + C_n_beta*beta + C_n_r*rhat + C_n_da*da + C_n_dr*dr;
C_x = 0;
C_y = C_y_beta*beta + C_y_dr*dr + C_y_p*phat + C_y_r*rhat;
C_z = 0;

Lift = C_L*Qa*S;
Drag = C_D*Qa*S;
%T0 = 5.0882;  %Apprentice_properties.m
% T0 = 0.889644; %(Newtons or 0.2 lb_f)
Thrust = T0 + C_x_dT*dT;
%--------------------------------------------------------------------%

Xa = Lift*sin(alpha) - Drag*cos(alpha) + C_x*Qa*S  + Thrust;
Ya = C_y*Qa*S;
Za = -Lift*cos(alpha) - Drag*sin(alpha) + C_z*Qa*S;
La = C_l*Qa*S*b;
Ma = C_m*Qa*S*c_bar;
Na = C_n*Qa*S*b;

XYZa = [Xa;Ya;Za];
LMNa = [La;Ma;Na];

%% Total FORCES AND MOMENTS HERE
g = 9.81;
FgravI = [0;0;m*g];
FgravB = T321'*FgravI;
XYZ = XYZa + FgravB + Fs;
LMN = LMNa + Ts;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%Translational Dynamics
p = pqr(1);
q = pqr(2);
r = pqr(3);
pqrskew = [0 -r q;r 0 -p;-q p 0];
uvwdot = (1/m)*XYZ - pqrskew*uvw;

%%%%Rotational Dynamics
pqrdot = Iinv*(LMN-pqrskew*I*pqr);

%dxdt = zeros(12,2);
dxdt = [xyzdot;ptpdot;uvwdot;pqrdot];