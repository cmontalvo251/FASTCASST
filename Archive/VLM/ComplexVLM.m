clear all
close all
clc
%%%%%%%%%%%%STATE VECTOR OF BODY%%%%%%%%%%%%%%%

%Body 1
x = 0;
y = 0;
z = -200;
phi = 0;
theta = 0.0;
psi = -0.3;
u = 20;
v = 0;
w = 1;
p = 0;
q = 0;
r = 0;

X1_cg = [x;y;z;phi;theta;psi;u;v;w;p;q;r];
ydx = 0;
yvec = 20:-1:3;
yvec = 3.04;
for y = yvec
ydx = ydx + 1;
%Body 2
x = 0
%y = 20.04;
y
z = -200;
phi = 0;
theta = 0.0;
psi = 0.3;
u = 20;
v = 0;
w = 1;
p = 0;
q = 0;
r = 0;

X2_cg = [x;y;z;phi;theta;psi;u;v;w;p;q;r];

X_cg = [X1_cg;X2_cg];

c = 0.3215;
%let c = 1/5
%c = 1/5;
wspan = 2.04;
%let wspan = 5*c
%wspan = 5*c;
mainwx = 0;

%%%%%%%%%%%GENERATE PANEL LOCATIONS%%%%%%%%%%%%

NPANELS = 3;
NBODIES = 2;
sweep = 0*pi/180;

r_cgp_B = zeros(3,NBODIES*NPANELS); %% position vector of panel from cg in body coordinates
r_cgA_B = zeros(3,NBODIES*NPANELS); %% position vector of point A of horshoe vortex in body coord
r_cgB_B = zeros(3,NBODIES*NPANELS); %% position vector of point B of horshoe vortex in body coord
E_cgp = zeros(3,NBODIES*NPANELS);  %% relative euler rotation of panel from body frame

%plottool(1,'Plane',12,'y','x')
for ii = 1:NBODIES
  for jj = 1:NPANELS
    loc = (ii-1)*NPANELS+jj;
    ycoord = -wspan/2 + wspan/(2*NPANELS) + (jj-1)*wspan/NPANELS;
    r_cgp_B(:,loc) = [mainwx-abs(ycoord)*tan(sweep)-3*c/4;ycoord;0];
    Acoord = ycoord-wspan/(2*NPANELS);
    r_cgA_B(:,loc) = [mainwx-abs(Acoord)*tan(sweep)-c/4;Acoord;0];
    Bcoord = ycoord+wspan/(2*NPANELS);
    r_cgB_B(:,loc) = [mainwx-abs(Bcoord)*tan(sweep)-c/4;Bcoord;0];
    % plot(r_cgp_B(2,loc),r_cgp_B(1,loc),'b*')
    % plot(r_cgA_B(2,loc),r_cgA_B(1,loc),'r*')
    % plot(r_cgB_B(2,loc),r_cgB_B(1,loc),'r*')
  end
end
%%%%%ASSUME INDUCED VELOCITY AT EACH PANEL%%%%%

uvw_induced_B = zeros(3,NBODIES*NPANELS);
u = X1_cg(7);
v = X1_cg(8);
w = X1_cg(9);
Vinf = sqrt(u^2+v^2+w^2);
alfa = atan2(w,u);
for ii = 1:NBODIES*NPANELS
    uvw_induced_B(:,ii) = [0;0;Vinf*alfa];
end

%%%%%DEFINE SOME REFERENCE FRAMES%%%%%%%%%

%%B = BODY FRAME
%%P = PANEL FRAME
%%I = INERTIAL FRAME

%%Transformation matrix from Body to inertial frame
TIB = zeros(3,3,NBODIES);
for ii = 1:NBODIES
    loc = 4 + (ii-1)*12;
    phi = X_cg(loc);
    theta = X_cg(loc+1);
    psi = X_cg(loc+2);
    TIB(:,:,ii) = R123(phi,theta,psi); 
end

%%%Transformation from panel to body frame
TBP = zeros(3,3,NPANELS*NBODIES);
for ii = 1:NPANELS*NBODIES
  TBP(:,:,ii) = R123(E_cgp(1,ii),E_cgp(2,ii),E_cgp(3,ii)); 
end

%COMPUTE VELOCITY AT EACH PANEL WITHOUT INCLUDING INDUCED VELOCITY%
Vcg_B = [u;v;w];
PQR = [0 -r q;r 0 -p;-q p 0];
VP_noInduced_B = zeros(3,NPANELS*NBODIES);
rp_I = zeros(3,NPANELS*NBODIES);
vortexA_I = zeros(3,NPANELS*NBODIES);
vortexB_I = zeros(3,NPANELS*NBODIES);
for ii = 1:NBODIES
  for jj = 1:NPANELS
    loc = (ii-1)*NPANELS+jj;
    s = (ii-1)*12+1;
    e = s+2;
    rp_I(:,loc) = X_cg(s:e) + TIB(:,:,ii)*r_cgp_B(:,loc); %%Compute location of panel in inertial space
    vortexA_I(:,loc) = X_cg(s:e) + TIB(:,:,ii)*r_cgA_B(:,loc); %%Compute location of pt A in inertial space
    vortexB_I(:,loc) = X_cg(s:e) + TIB(:,:,ii)*r_cgB_B(:,loc); %%Compute location of pt B in inertial space
    Vatm_I = AtmModel(rp_I(:,loc)); %%Compute atmospheric component at location of panel
    VP_noInduced_B(:,loc) = Vcg_B + PQR*r_cgp_B(:,loc) + TIB(:,:,ii)'*Vatm_I;
  end
end

%%%%%%%%%%%%%%%%NOW WE START THE ITERATION ON THE PANELS%%%%%%%%%%%%%%%%%%%
iterate = 1;
iternum = 1;
VP_P = zeros(3,NPANELS*NBODIES);
VP_B = zeros(3,NPANELS*NBODIES);
VP_I = zeros(3,NPANELS*NBODIES);
rho = 1.225;
S = 0.6558;
c_bar = 0.3215;
Spanel = S/NPANELS;
cpanel = c_bar;
Gammai = zeros(NPANELS*NBODIES,1);
Gammaiprev = Gammai;
itermax = 100;
Gammavec = zeros(NPANELS*NBODIES,itermax);
damping = 0.2;
C = zeros(NPANELS*NBODIES);
while iterate
  %iternum
  body = 1;
  for ii = 1:NPANELS*NBODIES
    if ii > NPANELS
        body = 2;
    end
    %%%%%%ADD IN THE CURRENT SOLUTION OF INDUCED VELOCITY%%%%%%%%%%
    VP_temp = VP_noInduced_B(:,ii);
    %VP_temp(3) = 0;
    VP_B(:,ii) = VP_temp - uvw_induced_B(:,ii);
    VP_I(:,ii) = TIB(:,:,body)*VP_B(:,ii);
    VP_P(:,ii) = TBP(:,:,ii)'*VP_B(:,ii);
    Vinf = norm(VP_P(:,ii));
    %%%%%Compute CL, CD at each panel%%%%%
    [CL,CD,CM] = AeroModel(VP_P(:,ii));
    %SectionLpanel = 1/2*rho*Vinf^2*cpanel*CL;
    %%%%%%Compute Gamma at each panel%%%%%%
    %Gammai = SectionLpanel/(rho*Vinf)
    Gammai(ii) = Vinf*cpanel*CL/2;
  end
  Gammavec(:,iternum) = Gammai;
  if abs(Gammai(1)-Gammaiprev(1)) < 1e-8 && iternum > 10
      iterate = 0;
  end
  Gammai = (1-damping)*Gammaiprev + damping*Gammai;  
  Gammaiprev = Gammai;
  %Gammai = [-0.8164;-0.8164];
  %%%%%%Recompute w_induced%%%%%%%%%%%
  %%This is a double sum around all the panels
  body = 1;
  for ii = 1:NPANELS*NBODIES
    %ii = control point
    uvw_induced_B(:,ii) = [0;0;0];
    if ii > NPANELS
        body = 2;
    end
    for jj = 1:NPANELS*NBODIES
      %%jj = current horshoe vortex  
      %%vortexA_I = x,y,z coordinates of A point of vortex
      %%vortexB_I = x,y,z coordinates of B point of vortex
      %%VP_I = u,v,w coordinates of panel in inertial space
      wij_I = Horseshoe(rp_I(:,ii),vortexA_I(:,jj),vortexB_I(:,jj),VP_I(:,jj),Gammai(jj));
      uvw_induced_B(:,ii) = uvw_induced_B(:,ii) + TIB(:,:,body)'*wij_I*Gammai(jj);
      C(ii,jj) = wij_I(3);
    end
  end
  %%%%%%Convergence condition%%%%%%
  iternum = iternum + 1;
  if iternum == itermax
    iterate = 0;
    Gammavec(:,iternum) = Gammai;
  end
end
Gammai
uvw_induced_B
%%%%TOTAL LIFT ON WING IS NOW X=SUM(CX) Y=SUM(CY) Z = SUM(CZ)
XYZp_P = zeros(3,1);
LMNp_P = zeros(3,1);
XYZp_B = zeros(3,1);
LMNp_B = zeros(3,1);
XYZ_B = zeros(3,1);
LMN_B = zeros(3,1);
body = 1;
Ltotal = 0;
for ii = 1:NPANELS
  if ii > NPANELS
      body = 2;
  end
  %%%%%%ADD IN THE CURRENT SOLUTION OF INDUCED VELOCITY%%%%%%%%%%
  VP_B(:,ii) = VP_noInduced_B(:,ii) - uvw_induced_B(:,ii);
  VP_I(:,ii) = TIB(:,:,body)*VP_B(:,ii);
  VP_P(:,ii) = TBP(:,:,ii)'*VP_B(:,ii);
  Vinf = norm(VP_P(:,ii));
  %%%%%Compute CL, CD at each panel%%%%%
  [CL,CD,CM] = AeroModel(VP_P(:,ii));
  CL
  Lpanel = 1/2*rho*Vinf^2*Spanel*CL;
  Dpanel = 1/2*rho*Vinf^2*Spanel*CD;
  Mpanel = 1/2*rho*Vinf^2*Spanel*CM*cpanel;
  alfa = atan2(VP_P(3,ii),VP_P(1,ii));
  XYZp_P(1,1) = Lpanel*sin(alfa) - Dpanel*cos(alfa);
  XYZp_P(2,1) = 0;
  XYZp_P(3,1) = -Lpanel*cos(alfa) - Dpanel*sin(alfa);
  XYZp_B = TBP(:,:,ii)*XYZp_P;
  LMNp_B = TBP(:,:,ii)*LMNp_P;
  XYZ_B = XYZ_B + XYZp_B;
  LMN_B = LMN_B + cross(r_cgp_B(:,ii),XYZp_B) + LMNp_B;
  Ltotal = Ltotal + Lpanel;
end
% figure();
% plot(Gammavec');
% figure();
% plot(rp_I(2,:),Gammai,'b*');
% drawnow
%%%%%%%%%%%%%%%%%%%%%%%%%END PANEL CODE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disp('Checks')
% %%w1=-Uinf*alfa
% disp('Check 1')
% C*4*pi
% disp('Check 2')
% alfa = atan2(uvw_induced_B(3,:),20);
% Uinfalfa = -20*alfa
% uvw_induced_B(3,:)
% disp('Check 3')
% D = C(5:end,5:end)+C(5:end,4:-1:1);
% D*4*pi
% disp('Check 4')
% Gamma4 = Gammai(5:end);
% alfa4 = alfa(5:end);
% D*Gamma4
% 20*alfa4'
% disp('Check 5')
% Gamma4'./(20.*alfa4*4*pi)
% 0.0273*4*pi*wspan*20*alfa4(1)
% Gamma4(1)
Ltotal
X(ydx) = XYZ_B(3);
end
%plot(yvec,X)