%%Ok let's examine splitting the aero up
%purge
close all

global wt vv

% x = 0;y=0;z=0;phi=0;theta=0;psi=0;

% u = 25;
% v = 5.7; %1
% w = 0.4; 
% p = -0.01;
% q = 0.01;
% r =  0.1;
% wt = 0;
% vv = 0;

% x=  269.7675;
% y=    1.1906;
% z= -200.0060;
% phi=   -0.0021;
% theta=    0.0533;
% psi=   -0.0017;
% u =   19.9767;
% v =  -0.0257;
% w =   1.0650;
% p =   0.0018;
% q =   -0.0000;
% r =   -0.0016;

%state = [x,y,z,phi,theta,psi,u,v,w,p,q,r];

x0 = [0 0 -200 0 0 0 23 0 0 0 0 0]';

state=[ -50.0000;
    4.0000;
 -200.0000;
    0.4418;
    0.1144;
    0.5492;
   22.7994;
   -0.8806;
    2.1358;
   -0.3903;
   -0.0182;
    2.3244];

state = x0;

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

da = 0;
dT = 0;
dr = 0;
de = 0;

controls = [da;dr;de;dT];

XYZa = zeros(6,21);
LMNa = zeros(6,21);
vec = zeros(1,21);
Bkdn = zeros(24,21);
wt = 0;
vv = 0;
for ii = 1:21
  r = 0+ii*20/21;
  vec(ii) = r;
  state = [x,y,z,phi,theta,psi,u,v,w,p,q,r];
  controls = [da;dT;dr;de];
  %First Split Aero
  [XYZa(1:3,ii),LMNa(1:3,ii),Bkdn(:,ii)] = Panels(state,controls);
  %Now Combined Aero
  [XYZa(4:6,ii),LMNa(4:6,ii)] = Point(state,controls);
end

%%What to plot
Names = {'X','Y','Z','L','M','N'};
for ii = 1:3
  plottool(1,Names{ii},12);
  plot(vec,Bkdn(ii,:),'g-','LineWidth',2)
  plot(vec,Bkdn(ii+3,:),'k-','LineWidth',2)
  plot(vec,Bkdn(ii+6,:),'m-','LineWidth',2)
  plot(vec,Bkdn(ii+9,:),'y-','LineWidth',2)
  plot(vec,XYZa(ii,:),'bx','LineWidth',2)
  plot(vec,XYZa(ii+3,:),'r-','LineWidth',2)
  legend('Wing','Body','Horizontal','Vertical','Panels','Point')
  plottool(1,Names{ii+3},12);
  plot(vec,Bkdn(ii+12,:),'g-','LineWidth',2)
  plot(vec,Bkdn(ii+15,:),'k-','LineWidth',2)
  plot(vec,Bkdn(ii+18,:),'m-','LineWidth',2)
  plot(vec,Bkdn(ii+21,:),'y-','LineWidth',2)
  plot(vec,LMNa(ii,:),'bx','LineWidth',2)
  plot(vec,LMNa(ii+3,:),'r-','LineWidth',2)
  % if ii == 1
  %   for ll = 1:length(vec)
  %     CL0i = 0.062;
  %     CLai = 3.486;
  %     CD0i = 0.0161;
  %     rho = 1.2256657849;
  %     CDai = 0.9084;
  %     CDa1 = CDai;
  %     CDa2 = CDai;
  %     CD01 = CD0i;
  %     CD02 = CD0i;
  %     CLa1 = CLai;
  %     CLa2 = CLai;
  %     CL01 = CL0i;
  %     CL02 = CL0i;
  %     b = 2.04;
  %     p = vec(ll);
  %     w1 = p*b/2;
  %     w2 = -p*b/2;
  %     u1 = u;
  %     u2 = u;
  %     V1 = sqrt(u1^2+v^2+w1^2);
  %     V2 = sqrt(u2^2+v^2+w2^2);
  %     alfa1 = atan2(w1,u1);
  %     alfa2 = atan2(w2,u2);
  %     S = 0.6558;
  %     S1 = S/2;
  %     S2 = S/2;
  %     CL1 = CL01 + CLa1*alfa1;
  %     CL2 = CL02 + CLa2*alfa2;
  %     CD1 = CD01 + CDa2*alfa1^2;
  %     CD2 = CD02 + CDa2*alfa2^2;
  %     L = 1/2*rho*(-CL1*cos(alfa1)-CD1*sin(alfa1))*V1^2*S1*b/2 - ...
  % 		   1/2*rho*(-CL2*cos(alfa2)-CD2*sin(alfa2))*V2^2*S2*b/2;
  %     mainwing(ll) = L;
  %   end
  %   plot(vec,mainwing,'k--')
  % end
  legend('Wing','Body','Horizontal','Vertical','Panels','Point')
end

