function [stateout,controls] = LinearD(t,state,timestep)
%%% Equations of motion for a stable fixed wing aircraft.
%%% Unwrap state and control vectors
global HP P M N K KCA KCAB

x = state(1);
y = state(2);
z = state(3);
theta = state(4);
psi = state(5);
u = state(6);
q = state(7);
r = state(8);


if timestep == 0.001
  A = [
    1.0000         0         0         0         0    0.0010         0         0;
         0    1.0000         0         0    0.0200         0         0    0.0000;
         0         0    1.0000   -0.0200         0         0   -0.0000         0;
         0         0         0    1.0000         0         0    0.0009         0;
         0         0         0         0    1.0000         0         0    0.0010;
         0         0         0         0         0    0.9999         0         0;
         0         0         0         0         0         0    0.7521         0;
         0         0         0         0         0         0         0    0.9990];
  B = [
                         0                         0                         0;
                         0                         0                         0;
                         0                         0                         0;
                         0                         0                         0;
                         0                         0                         0;
                         0                         0       0.00999959909175047;
       -0.0402849133101673                         0                         0;
                         0      0.000131337798485395                         0];
elseif timestep == 0.01
  A =[
    1.0000         0         0         0         0    0.0100         0         0;
         0    1.0000         0         0    0.2000         0         0    0.0010;
         0         0    1.0000   -0.2000         0         0   -0.0005         0;
         0         0         0    1.0000         0         0    0.0033         0;
         0         0         0         0    1.0000         0         0    0.0099;
         0         0         0         0         0    0.9992         0         0;
         0         0         0         0         0         0    0.0579         0;
         0         0         0         0         0         0         0    0.9897];
  B =[
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0    0.1000;
   -0.1531         0         0;
         0    0.0013         0];
elseif timestep == 0.1
  A = [
    1.0000         0         0         0         0    0.0996         0         0;
         0    1.0000         0         0    2.0000         0         0    0.0966;
         0         0    1.0000   -2.0000         0         0   -0.0068         0;
         0         0         0    1.0000         0         0    0.0035         0;
         0         0         0         0    1.0000         0         0    0.0950;
         0         0         0         0         0    0.9920         0         0;
         0         0         0         0         0         0    0.0000         0;
         0         0         0         0         0         0         0    0.9016];

  B = [
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0    0.9960;
   -0.1625         0         0;
         0    0.0125         0];
elseif timestep == 1
  A =[
    1.0000         0         0         0         0    0.9610         0         0;
         0    1.0000         0         0   20.0000         0         0    7.2831;
         0         0    1.0000  -20.0000         0         0   -0.0699         0;
         0         0         0    1.0000         0         0    0.0035         0;
         0         0         0         0    1.0000         0         0    0.6227;
         0         0         0         0         0    0.9229         0         0;
         0         0         0         0         0         0    0.0000         0;
         0         0         0         0         0         0         0    0.3548];
  B = [
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0    9.6096;
   -0.1625         0         0;
         0    0.0818         0];
end

%%CONTROLS
da = 0;
dr = 0;
de = 0;
dT = 0;
%%%%%%%%%MAPPING%%%%%%%
statenext = A*state;
xdot = (statenext(1)-state(1))/timestep;
ydot = (statenext(2)-state(2))/timestep;
thetadot = (statenext(4)-state(4))/timestep;
psidot = (statenext(5)-state(5))/timestep;
ct = cos(theta);
st = sin(theta);
cphi = 1;
sphi = 0;
cpsi = cos(psi);
spsi = sin(psi);
L2 = [ct 0 -st; 0 1 0; st 0 ct];
L3 = [cpsi spsi 0; -spsi cpsi 0; 0 0 1];
TH = (L2*L3)';

MSLOPE = 0;
MSLOPE2 = 0;
BINTERCEPT2 = 0;
BINTERCEPT = 0;
Vtrim = 20;
ARCLENGTHPARENT = timestep*Vtrim*HP;
lbar(1) = cos(MSLOPE);
lbar(2) = sin(MSLOPE);
planedirection = lbar(1)*(x-0) + lbar(2)*(y-BINTERCEPT);
normAC = sqrt(x*x+y*y);
flightdirection = sign(xdot*lbar(1)+ydot*lbar(2));
xs = 0 + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar(1);
ys = BINTERCEPT + (planedirection + flightdirection*ARCLENGTHPARENT)*lbar(2);
zs = -200-BINTERCEPT2;
rwpI = [xs - x;ys - y;zs - z];
rwpB = TH'*rwpI;
vwpB(1) = lbar(1)*xdot + lbar(2)*ydot;
vwpB(2) = -lbar(2)*xdot + lbar(1)*ydot;
vwpB(3) = st*u;
delpsi = 0.005*(rwpB(2)) - 0.02*(vwpB(2));
thetac = -0.005*(rwpB(3)) + 0.02*(vwpB(3));
uc = 0.9*(rwpB(1))+0.5*(vwpB(1));

%%%%COMPUTE CONTROL%%%
%%%%%%%%%%%%%%%%%%%%%%%
PID = 0;
LMPC = 1;

if PID
  qc = 1*(thetac-theta)+0*(-thetadot);
  rc = 1*(delpsi)+0*(-psidot);
  uc = 20;
  dT = 1*(uc-u);
  de = -5*(qc-q);
  dr = 5*(rc-r);
end
if LMPC
  YCOMMAND = zeros(P*(HP+1),1);
  %%%%GENERATE PATHS from x,y,z to xs,ys,zs%%
  % invC = [3 -1;-2 1];
  % c0 = x;
  % c1 = xdot;
  % d0 = y;
  % d1 = ydot;
  % xdots = 20;
  % ydots = 0;
  % c2c3 = invC*[xs-c0-c1;xdots-c1];
  % c2 = c2c3(1);
  % c3 = c2c3(2);
  % d2d3 = invC*[ys-d0-d1;ydots-d1];
  % d2 = d2d3(1);
  % d3 = d2d3(2);
  scoord = linspace(0,1,HP+1);
  % xcoord = c0 + c1.*scoord + c2.*scoord.^2 + c3.*scoord.^3;
  % ycoord = (d0 + d1.*scoord + d2.*scoord.^2 + d3.*scoord.^3);
  xcoord = x + scoord.*(xs-x);
  ycoord = y + scoord.*(ys-y);
  zcoord = z + scoord.*(zs-z);
  ucoord = 0.*zcoord;
  thetacoord = 0.*ucoord;
  psicoord = 0.*thetacoord;
  qcoord = 0.*thetacoord;
  rcoord = 0.*thetacoord;
  %%%%Use x,y,z path to compute u,theta and psi
  ucoord(1) = u;
  thetacoord(1) = theta;
  psicoord(1) = psi;
  qcoord(1) = q;
  rcoord(1) = r;
  YCOMMAND(1:3,1) = [u;q;r];
  for ii = 2:length(xcoord)
    p0 = sqrt((xcoord(ii-1)-xcoord(1))^2+(ycoord(ii-1)-ycoord(1))^2+(zcoord(ii-1)-zcoord(1))^2);
    p1 = sqrt((xcoord(ii)-xcoord(1))^2+(ycoord(ii)-ycoord(1))^2+(zcoord(ii)-zcoord(1))^2);
    ucoord(ii) = (p1-p0)/timestep;
    ucoord(ii) = Vtrim;
    thetacoord(ii) = -atan2(zcoord(ii)-zcoord(ii-1),p1-p0);
    psicoord(ii) = atan2(ycoord(ii)-ycoord(ii-1),xcoord(ii)-xcoord(ii-1));
    %%Now use psicoord and thetacoord to generate q and r    
    qcoord(ii) = (thetacoord(ii)-thetacoord(ii-1))/timestep;
    rcoord(ii) = (psicoord(ii)-psicoord(ii-1))/timestep;
    %%Now populate YCOMMAND
    YCOMMAND((ii-1)*P+1:(ii-1)*P+P,1)=[ucoord(ii);qcoord(ii);rcoord(ii)];
  end
  YCOMMAND(1:3) = [];
  U = K*(YCOMMAND-KCA*state);
  uc = U(1:3);
  de = uc(1);
  dr = uc(2);
  dT = uc(3);
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
uc = [de;dr;dT];

%%%%Limit Controls

stateout = A*state+B*uc;

