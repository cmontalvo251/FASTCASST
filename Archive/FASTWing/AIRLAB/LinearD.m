function [stateout,controls] = LinearD(t,state,timestep)
%%% Equations of motion for a stable fixed wing aircraft.
%%% Unwrap state and control vectors
global HP P M N K KCAB KCA timestepD A B nominal control0
global UCOMMAND ZCOMMAND MSLOPE BINTERCEPT ARCLENGTHPARENT

if timestep == 0.01
A =[1.0000         0         0    0.0100         0   -0.0005         0    0.0100         0    0.0006         0   -0.0000         0;
         0    1.0000         0         0   -0.0108         0    0.2022         0    0.0100         0    0.0000         0    0.0000;
         0         0    1.0000         0         0   -0.2022         0   -0.0006         0    0.0096         0   -0.0030         0;
         0         0         0    1.0000         0         0         0         0         0         0         0         0         0;
         0         0         0         0    1.0000         0         0         0   -0.0000         0    0.0079         0    0.0007;
         0         0         0         0         0    1.0000         0    0.0000         0   -0.0001         0    0.0033         0;
         0         0         0         0    0.0000         0    1.0000         0    0.0000         0   -0.0000         0    0.0100;
         0         0         0         0         0   -0.0979         0    0.9993         0    0.0062         0    0.0187         0;
         0         0         0         0    0.0978         0         0         0    0.9970         0    0.0109         0   -0.2004;
         0         0         0         0         0   -0.0050         0   -0.0065         0    0.9344         0   -0.3501         0;
         0         0         0         0   -0.0001         0         0         0   -0.0015         0    0.6144         0    0.0240;
         0         0         0         0         0   -0.0001         0    0.0016         0   -0.0078         0    0.0600         0;
         0         0         0         0    0.0000         0         0         0    0.0002         0   -0.0023         0    0.9896];
B = [    0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
   -0.0185         0    0.0136;
         0    0.0011         0;
    0.0597         0   -0.0000;
         0   -0.2134         0;
   -0.1535         0    0.0000;
         0   -0.0077         0];
end

%%Unwrap State
x = state(1);
dy = state(2);
dz = state(3);
u0 = state(4);
dphi = state(5);
dtheta = state(6);
dpsi = state(7);
du = state(8);
dv = state(9);
dw = state(10);
dp = state(11);
dq = state(12);
dr = state(13);

%%unwrap nominal
y0 = nominal(2);
z0 = nominal(3);
phi0 = nominal(4);
theta0 = nominal(5);
psi0 = nominal(6);
u0 = nominal(7);
v0 = nominal(8);
p0 = nominal(10);
q0 = nominal(11);
r0 = nominal(12);
z = z0+dz;
u = u0+du;
y = y0+dy;
p = p0+dp;
q = q0+dq;
r = r0+dr;
v = v0+dv;
phi = dphi + phi0;
theta = dtheta + theta0;
psi = dpsi + psi0;

%%CONTROLS
da = 0;
dr = 0;
de = 0;
dT = 0;

%%Define Some Variables
nocontrol = A*state;
xdot = (nocontrol(1)-state(1))/timestep;
ydot = (nocontrol(2)-state(2))/timestep;
zdot = (nocontrol(3)-state(3))/timestep;
cs = cos(psi);
ss = sin(psi);

%%%%COMPUTE CONTROL%%%
%%%%%%%%%%%%%%%%%%%%%%%
PID = 0;
NOMINALC = 0;
LMPC = 1;

lbar0 = cos(MSLOPE);
lbar1 = sin(MSLOPE);
planedirection = lbar0*(x-0) + lbar1*(y-BINTERCEPT);
normAC = sqrt(x*x+y*y);
flightdirection = sign(xdot*lbar0+ydot*lbar1);
xs = 0 + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar0;
ys = BINTERCEPT + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar1;
zs = ZCOMMAND;

if LMPC
  YCOMMAND = zeros(P*(HP+1),1);
  scoord = linspace(0,1,HP+1);
  xcoord = x + scoord.*(xs-x);
  ycoord = y + scoord.*(ys-y);
  zcoord = z + scoord.*(zs-z);
  ucoord = 0.*zcoord;
  thetacoord = 0.*ucoord;
  phicoord = 0.*ucoord;
  psicoord = 0.*thetacoord;
  pcoord = 0.*thetacoord;
  qcoord = 0.*thetacoord;
  ucoord(1) = u;
  thetacoord(1) = theta;
  psicoord(1) = psi;
  phicoord(1) = phi;
  qcoord(1) = q;
  rcoord(1) = r;
  %%Now populate YCOMMAND
  for ii = 2:length(xcoord)
    p1 = sqrt((xcoord(ii-1)-xcoord(1))^2+(ycoord(ii-1)-ycoord(1))^2+(zcoord(ii-1)-zcoord(1))^2);
    p2 = sqrt((xcoord(ii)-xcoord(1))^2+(ycoord(ii)-ycoord(1))^2+(zcoord(ii)-zcoord(1))^2);
    ucoord(ii) = (p2-p1)/timestep;
    ucoord(ii) = UCOMMAND;
    thetacoord(ii) = -atan2(zcoord(ii)-zcoord(ii-1),p2-p1);
    psicoord(ii) = atan2(ycoord(ii)-ycoord(ii-1),xcoord(ii)-xcoord(ii-1));
    phicoord(ii) = -3*(psi-psicoord(ii));
    if phicoord(ii) > 45*pi/180
      phicoord(ii) = sign(phicoord(ii))*45*pi/180;
    end
    %%Now use phicoord and thetacoord to generate p and q
    pcoord(ii) = (phicoord(ii)-phicoord(ii-1))/timestep;
    qcoord(ii) = (thetacoord(ii)-thetacoord(ii-1))/timestep;
    %%Now substract nominal values to get deltas
    duii = ucoord(ii) - u0;
    dpii = pcoord(ii) - p0;
    dqii = qcoord(ii) - q0;
    YCOMMAND((ii-1)*P+1:(ii-1)*P+P,1)=[duii;dpii;dqii];
  end
  YCOMMAND(1:3) = [];
  U = K*(YCOMMAND-KCA*state);
  uc = U(1:3);
  de = uc(1);
  da = uc(2);
  dT = uc(3);
  KV = 0.04;
  dr = -KV*v;
  %%%PID DEBUG
  %dT = 0.2*(UCOMMAND-u);
  %zerror = z-ZCOMMAND;
  %de = -2.0*zerror-0.3*zdot;
  % rcpath0 = xs - x;
  % rcpath1 = ys - y;
  % rcpathPsi0 = cs*rcpath0 + ss*rcpath1;
  % rcpathPsi1 = -ss*rcpath0 + cs*rcpath1;
  % vpcpathPsi0 = cos(MSLOPE)*xdot + sin(MSLOPE)*ydot;
  % vpcpathPsi1 = -sin(MSLOPE)*xdot + cos(MSLOPE)*ydot;
  % delpsi = -0.005*rcpathPsi1 + 0.02*vpcpathPsi1;
  % PSICOMMANDbody = psi - delpsi;
  % phipathcommand = -3*delpsi;
  % if (abs(phipathcommand) > 45*pi/180)
  %   phipathcommand = sign(phipathcommand)*45*pi/180;
  % end
  % da = 1.2*(phi-phipathcommand)+0.1*p;
end
if NOMINALC
  da = 0;
  dr = 0;
  de = -0.06103;
  dT = 0;
end

if PID
  dT = 0.2*(UCOMMAND-u);
  zerror = z-ZCOMMAND;
  de = -2.0*zerror-0.3*zdot;
  rcpath0 = xs - x;
  rcpath1 = ys - y;
  rcpathPsi0 = cs*rcpath0 + ss*rcpath1;
  rcpathPsi1 = -ss*rcpath0 + cs*rcpath1;
  vpcpathPsi0 = cos(MSLOPE)*xdot + sin(MSLOPE)*ydot;
  vpcpathPsi1 = -sin(MSLOPE)*xdot + cos(MSLOPE)*ydot;
  delpsi = -0.005*rcpathPsi1 + 0.02*vpcpathPsi1;
  PSICOMMANDbody = psi - delpsi;
  phipathcommand = -3*delpsi;
  if (abs(phipathcommand) > 45*pi/180)
    phipathcommand = sign(phipathcommand)*45*pi/180;
  end
  da = 1.2*(phi-phipathcommand)+0.1*p;
  KV = 0.04;
  dr = -KV*v;
end


if abs(de) > 30*pi/180
  de = sign(de)*30*pi/180;
end
if abs(dr) > 30*pi/180
  dr = sign(dr)*30*pi/180;
end
if abs(da) > 30*pi/180
  da = sign(da)*30*pi/180;
end

controls = [da;dr;de;dT];
de0 = control0(3);
da0 = 0;
dT0 = 0;
duc = [de-de0;da-da0;dT-dT0];

stateout = A*state+B*duc;

