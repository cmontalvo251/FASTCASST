function [state_dot,controls] = SixDOF_Derivative(t,state,controls)
%%%%% Equations of motion for a stable fixed wing aircraft.
%%%%%% Unwrap state and control vectors
global udot wdot

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

de = controls(1);
da = controls(2);
dT = controls(3);
dr = controls(4);

%%% Mass Moment of Inertia Matrix in Body Axis (slug.ft^2)
%Small UAV
% Ixx = 0.4923; OLD
% Iyy = 0.9546; OLD
% Izz = 1.3766; OLD
Ixx = 0.4497;
Iyy = 0.5111;
Izz = 0.8470;
Ixy = 0;
Ixz = 0;
Iyz = 0;
Weight = 55.07;
%%Etkins Book
% mui = 272;
% Iyhat = 7*mui;
% rhoe = 0.000889; %%slug/ft^3
% rhoENG = rhoe;
% rhoSI = 14.5939*(rhoENG)*((3.28084)^3);
% Se = 1667; %%ft^2
% c_bare = 15.40; %ft
% IyENG = Iyhat*rhoe*Se*(c_bare/2)^3; %%slug-ft^2
% Iy = 14.594*IyENG/(3.28^2); %convert to kg-m^2
% Ix = 14.594*(855000)/(3.28^2); %convert to kg-m^2
% Iz = 14.594*(2160000)/(3.28^2); %convert to kg-m^2
% Weight = 9.81*14.59*(100000/32.2); %%lb to N
%%Emily's A/C
% mass = 0.83608352170; %[kg]
% Weight = mass*9.81;
% Ixx = 0.009279213296184;
% Iyy = 0.005660930419743;
% Izz = 0.014377713212214;
% Ixy = 0;
% Ixz = -7.893169818013876e-005;
% Iyz = -3.252606517456513e-019;

%%Compute Moments of Inertia and mass
grav = 9.81;
I_b = [Ixx Ixy Ixz;Ixy Iyy Iyz;Ixz Iyz Izz];
I_b_inv = inv(I_b);
mass = Weight/grav;

%%% Trig functions
ct = cos(theta);
st = sin(theta);
cp = cos(phi);
sp = sin(phi);
cs = cos(psi);
ss = sin(psi);

%%% Kinematics
L1 = [1 0 0; 0 cp sp; 0 -sp cp];
L2 = [ct 0 -st; 0 1 0; st 0 ct];
L3 = [cs ss 0; -ss cs 0; 0 0 1];
TH = (L1*L2*L3)';
xyz_dot = TH*[u; v; w];
xdot = xyz_dot(1);
ydot = xyz_dot(2);
zdot = xyz_dot(3);
pts_dot = [1, sp*st/ct, cp*st/ct; 0, cp, -sp; 0, sp/ct, cp/ct]*[p; q; r];

%AeroRoutine
[Xa,Ya,Za,La,Ma,Na] = Point(state,controls);
%[Xa,Ya,Za,La,Ma,Na] = Panels(state,controls);

% AeroForceMom = RigidAerosplit4_carlos(0,state,de,da,dr);
% PropForceMom = RigidProp_carlos(state,dT);
% Xa = AeroForceMom(1) + PropForceMom(1);
% Ya = AeroForceMom(2)+ PropForceMom(2);
% Za = AeroForceMom(3)+ PropForceMom(3);
% La = AeroForceMom(4)+ PropForceMom(4);
% Ma = AeroForceMom(5)+ PropForceMom(5);
% Na = AeroForceMom(6)+ PropForceMom(6);

%%%WEIGHT

GW = mass*grav;
Xw = -GW*st;
Yw = GW*sp*ct;
Zw = GW*cp*ct;  

%%BUNDLE UP DERIVATIVES

X = Xa + Xw;
Y = Ya + Yw;
Z = Za + Zw;

L = La;
M = Ma;
N = Na;

%%% Dynamics
uvw_dot = [X; Y; Z]/mass + [r*v-q*w;p*w-r*u;q*u-p*v];
pqr_dot = I_b_inv*([L; M; N]-[0 -r q;r 0 -p;-q p 0]*I_b*[p; q; r]);

state_dot = [xyz_dot; pts_dot; uvw_dot; pqr_dot];
