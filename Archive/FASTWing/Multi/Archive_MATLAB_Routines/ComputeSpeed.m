function [wnq,kpz,wnp,kppsi,qmax,pmax] = ComputeSpeed()

close all

global S I_b c_bar b mass I_b_inv T0 num_ac rcg2ac phic thetac

addpath 'plotting/'

%%FLAGS
SDOF = 1;
NDOF = 0;

%%Time Stuff
t0 = 0;
tf = 2;
timestep = 0.01;

%%%%6DOF%%%
col = 1;
%disp('6DOF Model')
%for timestep = [0.1 0.01 0.001]
%col = col + 1;
SetupConfiguration
%%Scale aircraft
%scale(1);
%%Initial Conditions
x0 = [xcg ycg -200 0 0 0 20 0 0 0 0 0 0 0]';
metacontrols = zeros(4*num_ac,1);
metacontrols(1) = 42;
[tout,xout,uout] = odeK4(@ACdot,[t0 tf],x0,timestep,metacontrols,100);
xout = xout';

%%%%Solve for QMAX
q = xout(:,11);
qmax = abs(max(q));
p = xout(:,10);
pmax = abs(max(p));
% plottool(1,'Q',12);
% plot(q)
% plottool(1,'P',12);
% plot(p)
% de = uout(3,:)*180/pi;
% plottool(1,'DE',12);
% plot(de)
% da = uout(1,:)*180/pi;
% plottool(1,'DA',12);
% plot(da)

%%%Compute Rise time
tr = (30*pi/180)./qmax;

%%%Compute natural frequency based on first order system
r = 0.98;
p = 1-r;
%%%%Assuming that zeta = 1
wnq = -log(p/5)./tr;

%%Compute Rise time of z
trz = 2*tr;
kpz = -log(p)/trz;

%%%Compute Rise time
tr = (10*pi/180)./pmax;

%%%Compute natural frequency based on first order system
r = 0.98;
p = 1-r;
%%%%Assuming that zeta = 1
wnp = -log(p/5)./tr;

%%Compute Rise time of z
trpsi = 2*tr;
kppsi = -log(p)/trpsi;

%Single aircraft qmax = 4.11 rad/s
%2-T2T - 0.4389 rad/s
%3-T2T - 0.2606 rad/s
%4-T2T - 0.1737 rad/s
%10-T2T - 0.0655 rad/s

function [state_dot,metacontrols] = ACdot(t,state,e)
%%%%% Equations of motion for a stable fixed wing aircraft.
%%%%%% Unwrap state and control vectors
global mass I_b I_b_inv num_ac rcg2ac T0 b c_bar S phic thetac

Vtrim = 20;

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
zint = state(13);

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

kinematicderivs = [xyz_dot;pts_dot];

controls = [0;0;0;0];

grav = 9.81;
GW = mass*grav;
Xw = -GW*st;
Yw = GW*sp*ct;
Zw = GW*cp*ct;  

X = Xw;
Y = Yw;
Z = Zw;
L = 0;
M = 0;
N = 0;

%%%Saturation on outer loop
L0 = 0;
M0 = 0;
Ixx = I_b(1,1);Iyy = I_b(2,2);Izz = I_b(3,3);
[L0,M0,N0,Lift0,Drag0,Y0,G] = ComputeFG(state,rcg2ac,num_ac,Ixx,Iyy);
F = [0;0];
F(1) = (1/Ixx)*(L0 + r*q*(Iyy-Izz));
F(2) = (1/Iyy)*(M0 + r*p*(Izz-Ixx));
%%%Define Psuedo Control
ptdbldotc = [0;0];
ptdot = [pts_dot(1);pts_dot(2)];
ptdotc = [0;0];
pt = [phi;theta];
%%%%%Use gains from simulating maximum control deflection
kptheta = 10000;
kdtheta = 0;
%%%%%Do the same for roll control
kpphi = 10000;
kdphi = 0;
%kd = [0.8 0;0 1.1];
%kp = [1.2 0;0 1.2];
kd = [kdphi 0;0 kdtheta];
kp = [kpphi 0;0 kptheta];
ptc = [phic;thetac];
gammax = ptdbldotc - kd*(ptdot-ptdotc) - kp*(pt-ptc);
%%%Compute Feedbacklinearization Control
uFL = gammax - F;
%%%Compute Control allocation
metadade = G'*inv(G*G')*uFL;
%metadade = inv(G)*uFL;

%%%Loop until none or all controls are saturated
issat = 1;
u0 = uFL;
G0 = G;
counter = 1:length(metadade);
sat_controls = [];
l = 0;
while issat
    %%%Check for saturation
    sat_controls = [sat_controls;find(abs(metadade) > 30*pi/180)];
    not_sat = counter;
    not_sat(sat_controls) = [];
    if length(sat_controls) == l || isempty(sat_controls)
        issat = 0;
    else
        %%%We must now set the saturated controls to the value of
        %%%saturation
        metadade(sat_controls) = sign(metadade(sat_controls))*30*pi/180;
        %%%Then we need to hit each one of these controls with the G matrix
        %%%and substract it from u0;
        uFL = u0 - G0(:,sat_controls)*metadade(sat_controls);
        %%%Then we need to reduce the order of G
        G = G0;
        G(:,sat_controls) = [];
        GG = G*G';
        %%%%Check to make sure that G*G' is not undefined
        if det(GG) < 1e-8
            issat = 0;
        else
            %%%Then recompute controls
            reduced = G'*inv(GG)*uFL;
            %%%Then repopulate the controls with the updated version
            metadade(not_sat) = reduced;
            %%%Save number of saturated controls
            l = length(sat_controls);
        end
    end
end

AERO = 1;
metacontrols = [];
if AERO
 for ii = 1:num_ac
    s = 1 + (ii-1)*2;
    da = metadade(s);
    de = metadade(s+1);
    controls = [da;0;de;0];
    [XYZa,LMNa,controls] = Point(state,controls,t,rcg2ac(:,ii),kinematicderivs);
    metacontrols = [metacontrols;controls];
    X = X + XYZa(1);
    Y = Y + XYZa(2);
    Z = Z + XYZa(3);
    L = L + LMNa(1);
    M = M + LMNa(2);
    N = N + LMNa(3);
 end
end


%%% Dynamics
uvw_dot = [X; Y; Z]/mass + [r*v-q*w;p*w-r*u;q*u-p*v];
pqr_dot = I_b_inv*([L; M; N]-[0 -r q;r 0 -p;-q p 0]*I_b*[p; q; r]);
zintdot = 0;
psiintdot = 0;
state_dot = [xyz_dot; pts_dot; uvw_dot; pqr_dot;zintdot;psiintdot];
