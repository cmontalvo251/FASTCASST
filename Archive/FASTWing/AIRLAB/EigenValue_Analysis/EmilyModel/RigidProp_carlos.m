function PropForceMom = RigidProp_carlos(state, dthrot)

% global MASS GRAVITY DENSITY AREA 
DENSITY = 1.225; %[kg/m^3]
% Unwrap State Vector
x = state(1); y = state(2); z = state(3);
phi = state(4); theta = state(5); psi = state(6);
u = state(7); v = state(8); w = state(9);
p = state(10); q = state(11); r = state(12);
xprop = 0.009525; %stationline of prop
yprop = 0.00; %buttline of prop
zprop = 0.00; %waterline of prop

    x_cg = -0.106175045947113;
    y_cg = 0;
    z_cg = -0.008781645909422;

%Find total velocity
%Assuming no wind
v_prop = [u;v;w] + [0 -r q; r 0 -p; -q p 0]*[xprop-x_cg; yprop-y_cg; zprop-z_cg];

v_axial = v_prop(1); %assuming prop not mounted at an angle

%Find RPM
rpm = 5333.333*dthrot + 279.1960*v_axial; %[rev/min]
omega = rpm*2*pi/60; %[rad/s]
diam = 0.2413; %propeller diameter [m]

J = v_axial/((rpm/60)*diam);  %Advance Ratio

%Read in advance ratio/efficiency/C_T table and interpolate to find
%power and thrust coefficients
proptab = dlmread('proptable.txt');
Jtab = proptab(:,1);
efftab = proptab(:,2);
eff = interp1(Jtab, efftab, J);
%Find Thrust
T = 7.6509*dthrot + -0.177791*v_axial;
if T < 0
    T = 0;
end
%Find Torque
Q = (v_axial*T/(omega*eff));
if omega < 0.001 || eff < 0.001
    Q = 0;
end
%  Q = 0; 


% omega
% v_axial
% J
% eff
% Q
% pause
rcgprop = [xprop-x_cg; yprop-y_cg; zprop-z_cg];
%Assuming thrust is only in x-direction and in-line with the c.g.:
X_P = T;
Y_P = 0; Z_P = 0; 

LMN_P = [-Q; 0;0] +cross(rcgprop,[X_P;Y_P;Z_P]);
L_P = LMN_P(1);
M_P = LMN_P(2);
N_P = LMN_P(3);


PropForceMom = [X_P; Y_P; Z_P; L_P; M_P; N_P];