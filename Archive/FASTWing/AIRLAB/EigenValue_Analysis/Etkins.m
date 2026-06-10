purge
format long g
%%First we start with the Modes computed from Nonlinear Simulation
files = {'Single_Aircraft_Single_Computation_Point.txt'};
Alinear = dlmread(['Linear_Matrices/',files{1}]);
[r,c] = size(Alinear);
NSTATE = 15;
num_ac = round(r/NSTATE);

Alinear = Alinear(1:(num_ac*NSTATE),1:(num_ac*NSTATE));

delrows = [];
angles = [];
for jj = 1:num_ac
  s = (jj-1)*NSTATE;
  s2 = (jj-1)*12;
  delrows = [delrows,(s+13):(s+12+3)];
  angles = [angles,(s2+4):(s2+6)];
end
Alinear(delrows,:) =  [];
Alinear(:,delrows) =  [];
acmodes = eig(Alinear)
[eigenvectors,singularvalues] = eig(Alinear);

xnominal = dlmread(['Linear_Matrices/',files{1}(1:end-4),'.nom']);
xnominal = xnominal(:,1);
xnominal(delrows)=[];

%%%THen we proceed with Etkins formulation

%%%We must get all values of aerodynamics into the form of Etkins
Ix = 0.4923;
Iy = 0.9546;
Iz = 1.3766;
I_b = [Ix 0 0;0 Iy 0;0 0 Iz];
I_b_inv = [1/Ix 0 0;0 1/Iy 0;0 0 1/Iz];
grav = 9.81;
Weight = 55.07;
mass = Weight/grav;
C_m_0 = 0.0598;
C_L_0 = 0.062;
C_D_0 = 0.028;
C_L_alpha = 5.195;
C_L_alpha_dot = 0;
C_D_alpha = 1.3537;
C_m_alpha = -0.9317;
C_m_alpha_dot = 0;
C_L_q = 4.589;
C_m_q = -5.263;
C_L_de = 0.2167;
C_m_de = -0.8551;
C_x_dT =  7.6509;
C_D_u = 0;
C_m_u = 0;
CDv = 0;
CLv = 0;
CmV = 0;

%%% Lateral Coefficients

C_y_beta = -0.2083;
C_l_beta = -0.0377;
C_n_beta = 0.0116;
C_l_p = -0.4625;
C_n_p = -0.0076;
C_l_r = 0.0288;
C_n_r = -0.0276;
C_l_da = -0.2559;
C_n_da = -0.0216;
C_y_dr = 0.1096;
C_l_dr = 0.0085;
C_n_dr = 0.0035;
C_y_p = 0.0057;

% rho 
rho = 1.220281;
% Reference Area (m^2)
S = 0.6558;
% Wingspan (m)
b = 2.04;
%Mean chord
c_bar = 0.3215;
%Trim Velocity
ue = xnominal(7);
ve = xnominal(8);
we = xnominal(9);
qe = xnominal(11);
Ve = sqrt(ue^2+ve^2+we^2);
qinf = 0.5*rho*(Ve^2)*S;
alphae = atan2(we,ue);
%Trim Control
dt = 0.003221;
de = 0.011753;
%Trim Thrust
T0 = 5.0882;
Te = T0 + C_x_dT*dt;
C_T = Te/(qinf);
dTdV = 0;

%176 Nondimensional forms
%186 longitudinal equations full
%188 Lateral equations full
%290 longitudinal derivs
%317 lateral derivs
%321 longitudinal A matrix

%%Etkins formulation
CTv = (dTdV)/qinf - 2*C_T;
mui = mass/(rho*S*c_bar/2);
Cw = Weight/(qinf);
Iyhat = Iy/(rho*S*(c_bar/2)^3);
C_De = C_D_0 + C_D_alpha*(pow(alphae,2)) + C_D_u*ue;
C_Le = C_L_0 + C_L_alpha*alphae + C_L_q*qe + C_L_de*de;

%%Example parameters
% purge
% CTe = 0.0188;
% C_De = CTe;
% CTv = -2*CTe;
% Cw = 0.25;
% mui = 272;
% hnh = 0.15;
% C_m_alpha = -4.88*(hnh);
% C_m_q = -22.9;
% C_L_alpha = 4.88;
% Iyhat = 7*mui;
% C_m_alpha_dot = -4.20;
% CDv = 0;
% C_Le = 0.25;
% C_D_alpha = 2*C_Le*C_L_alpha/(7*pi);
% CLv = 0;
% C_L_alpha_dot = 0;
% C_L_q = 0;
% CmV = 0;
% Ve = 733/3.28; %ft/s converted to m/s
% c_bar = 15.40/3.28; %ft converted to m

%%We start with using the approximation to the phugoid mode
phugoid = CTv/(4*mui) + i*(1/sqrt(2))*Cw/mui

%%Then we move to the short period mode
c0 = -(2*mui*C_m_alpha + C_m_q*(C_L_alpha+C_De))/(2*mui*Iyhat);
c1 = (Iyhat*(C_L_alpha + C_De)-2*mui*(C_m_q+C_m_alpha_dot))/(2*mui*Iyhat);

shortperiod = roots([1 c1 c0])

%Finally we contruct the longitundinal A matrix from Etkins and
%compute the Eigenvalues
Ahat = [(CTv-CDv)/(2*mui),(C_Le-C_D_alpha)/(2*mui),0,-Cw/(2*mui);-(CLv+2*Cw)/(2*mui+C_L_alpha_dot),-(C_L_alpha+C_De)/(2*mui+C_L_alpha_dot),(2*mui-C_L_q)/(2*mui+C_L_alpha_dot),0;(1/Iyhat)*(CmV-C_m_alpha_dot*(CLv+2*Cw)/(2*mui+C_L_alpha_dot)),(1/Iyhat)*(C_m_alpha-C_m_alpha_dot*(C_L_alpha+C_De)/(2*mui+C_L_alpha_dot)),(1/Iyhat)*(C_m_q+C_m_alpha_dot*(2*mui-C_L_q)/(2*mui+C_L_alpha_dot)),0;0,0,1,0]

%%%The matrix above is in nondimensional units thus the eigenvalues
%will be in non-dimensional units. Thus we must convert Ahat to
%dimensional units
rvhat = Ahat(1,:);
ralfahat = Ahat(2,:);
rqhat = Ahat(3,:);
rthetahat = Ahat(4,:);

rv = rvhat.*(2*Ve^2)/c_bar;
ralfa = ralfahat;
rq = rqhat.*(4*Ve^2)/(c_bar^2);
rtheta = rthetahat;

Astar = [rv;ralfa;rq;rtheta]

Vv = Astar(:,1);
Valfa = Astar(:,2);
Vq = Astar(:,3);
Vtheta = Astar(:,4);

A = [Vv./Ve Valfa Vq.*(c_bar/(2*Ve)) Vtheta]

Ahatmodes = eig(Ahat)
Amodes = eig(A)

