function [state_dot,controls] = LinearM(t,state,e)
%%% Equations of motion for a stable fixed wing aircraft.
%%% Unwrap state and control vectors
global HP P M N K KCAB KCA timestepD A B
global UCOMMAND ZCOMMAND MSLOPE BINTERCEPT ARCLENGTHPARENT
global nominal control0

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

A =[     0         0         0    1.0000         0    0.0000         0    0.9984         0    0.0560         0         0         0;
         0         0         0         0   -1.1321         0   20.2237         0    1.0000         0         0         0         0;
         0         0         0         0         0  -20.2237         0   -0.0560         0    0.9984         0         0         0;
         0         0         0         0         0         0         0         0         0         0         0         0         0;
         0         0         0         0         0         0         0         0         0         0    1.0000         0    0.0561;
         0         0         0         0         0         0         0         0         0         0         0    1.0000         0;
         0         0         0         0         0         0         0         0         0         0         0         0    1.0016;
         0         0         0         0         0   -9.7946         0   -0.0756         0    0.6792         0    6.2466         0;
         0         0         0         0    9.7946         0         0         0   -0.2958         0    1.2959         0  -20.1920;
         0         0         0         0         0   -0.5492         0   -0.5530         0   -7.4400         0 -111.4091         0;
         0         0         0         0         0         0         0         0   -0.1963         0  -48.7003         0    3.0326;
         0         0         0         0         0         0         0    0.4607         0   -2.4798         0 -285.7999         0;
         0         0         0         0         0         0         0         0    0.0216         0   -0.2862         0   -1.0393];
B =[     0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
    0.3484         0    1.3629;
         0         0         0;
   -6.2144         0         0;
         0  -26.9458         0;
  -46.4350         0         0;
         0   -0.8134         0];
%%CONTROLS
da = 0;
dr = 0;
de = 0;
dT = 0;

%%Define Some Variables
nocontrol = A*state;
xdot = nocontrol(1);
ydot = nocontrol(2);
zdot = nocontrol(3);
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
    ucoord(ii) = (p2-p1)/timestepD;
    ucoord(ii) = UCOMMAND;
    thetacoord(ii) = -atan2(zcoord(ii)-zcoord(ii-1),p2-p1);
    psicoord(ii) = atan2(ycoord(ii)-ycoord(ii-1),xcoord(ii)-xcoord(ii-1));
    phicoord(ii) = -3*(psi-psicoord(ii));
    if phicoord(ii) > 45*pi/180
      phicoord(ii) = sign(phicoord(ii))*45*pi/180;
    end
    %%Now use phicoord and thetacoord to generate p and q
    pcoord(ii) = (phicoord(ii)-phicoord(ii-1))/timestepD;
    qcoord(ii) = (thetacoord(ii)-thetacoord(ii-1))/timestepD;
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
  lbar0 = cos(MSLOPE);
  lbar1 = sin(MSLOPE);
  planedirection = lbar0*(x-0) + lbar1*(y-BINTERCEPT);
  normAC = sqrt(x*x+y*y);
  flightdirection = sign(xdot*lbar0+ydot*lbar1);
  xs = 0 + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar0;
  ys = BINTERCEPT + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar1;
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
state_dot = A*state+B*duc;

