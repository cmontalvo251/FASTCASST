purge

RUNCODE = 1;

if RUNCODE
  system('rm Run.exe');
  system('make rebuild');
  system('Run.exe source/MultiMeta.ifiles');                    
end

%%% Longitudinal Coefficients
C_m_0 = 0.0598;
C_L_0 = 0.062;
C_D_0 = 0.028;
C_D_q = 0;
C_L_alpha = 5.195;
C_L_alpha_hat = 0;
C_m_alpha = -0.9317;
C_m_alpha_hat = 0;
C_L_q = 4.589;
C_m_q = -5.263;
C_L_de = 0.2167;
C_m_de = -0.8551;
C_x_dT =  7.6509;
C_D_u = 0;
C_m_u = 0;

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
C_y_r = 0.0645;

%%z Geometry and Inertias

% % Wingspan (m)
b = 2.04;
% %Mean chord
c_bar = 0.3215;
% Reference Area (m^2)
S = c_bar*b;
%Trim Velocity
Vtrim = 20;
%Gravity
g = 9.81;
%Mass
mass = 5.6;
%Weight
W = mass*g;
%Density
rho = 1.220281;
Ix1 = 0.4497;
Iy1 = 0.5111;
Iz1 = 0.8470;

%%%C_D_alpha approximation
AR = b^2/S;
C_D_alpha = (C_L_alpha^2)/(pi*AR);

%%%ANGLE OF ATTACK SWEEPS
LDM = dlmread('source/Out_Files/AOA_Sweeps.FIT');

alfa = LDM(:,1).*180/pi;
Lift = LDM(:,2);
Drag = LDM(:,3);
Pitch_Moment = LDM(:,4);

%%%Ideal
alpha_Ideal = linspace(-10*pi/180,10*pi/180,100);
Lift_Ideal = 0.5*rho*Vtrim^2*S*(C_L_0 + C_L_alpha.*alpha_Ideal);
Drag_Ideal = 0.5*rho*Vtrim^2*S*(C_D_0 + C_D_alpha.*alpha_Ideal.^2);
Pitch_Moment_Ideal = 0.5*rho*Vtrim^2*S*c_bar*(C_m_0 + C_m_alpha.*alpha_Ideal);
f = 180/pi;

plottool(1,'L(AOA)',18,'Angle of Attack(deg)','Lift(N)');
plot(alfa,Lift,'b-','LineWidth',2)
plot(f.*alpha_Ideal,Lift_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'D(AOA)',18,'Angle of Attack(deg)','Drag(N)');
plot(alfa,Drag,'b-','LineWidth',2)
plot(f.*alpha_Ideal,Drag_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'M(AOA)',18,'Angle of Attack(deg)','Pitch Moment(N)');
plot(alfa,Pitch_Moment,'b-','LineWidth',2)
plot(f.*alpha_Ideal,Pitch_Moment_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

%%%PITCH RATE SWEEPS
LDM = dlmread('source/Out_Files/Pitch_Sweeps.FIT');

q = LDM(:,1);
Lift = LDM(:,2);
Pitch_Moment = LDM(:,3);

%%%Ideal
q_Ideal = linspace(-1,1,100);
q_hat = c_bar.*q_Ideal./(2*Vtrim);
Lift_Ideal = 0.5*rho*Vtrim^2*S*(C_L_0 + C_L_q.*q_hat);
Pitch_Moment_Ideal = 0.5*rho*Vtrim^2*S*c_bar*(C_m_0 + C_m_q.*q_hat);

plottool(1,'L(q)',18,'Pitch Rate(rad/s)','Lift(N)');
plot(q,Lift,'b-','LineWidth',2)
plot(q_Ideal,Lift_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'M(q)',18,'Pitch Rate(rad/s)','Pitch Moment(N)');
plot(q,Pitch_Moment,'b-','LineWidth',2)
plot(q_Ideal,Pitch_Moment_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

%%%ROLL RATE SWEEPS
LDM = dlmread('source/Out_Files/Roll_Sweeps.FIT');

p = LDM(:,1);
Y = LDM(:,2);
Roll_Moment = LDM(:,3);
Yaw_Moment = LDM(:,4);

%%%Ideal
p_Ideal = linspace(-1,1,100);
p_hat = b.*p_Ideal./(2*Vtrim);
Y_Ideal = 0.5*rho*Vtrim^2*S*(C_y_p.*p_hat);
Roll_Moment_Ideal = 0.5*rho*Vtrim^2*S*b*(C_l_p.*p_hat);
Yaw_Moment_Ideal = 0.5*rho*Vtrim^2*S*b*(C_n_p.*p_hat);

plottool(1,'Y(p)',18,'Roll Rate(rad/s)','Y(N)');
plot(p,Y,'b-','LineWidth',2)
plot(p_Ideal,Y_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'L(p)',18,'Roll Rate(rad/s)','Roll Moment(N-m)');
plot(p,Roll_Moment,'b-','LineWidth',2)
plot(p_Ideal,Roll_Moment_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'N(p)',18,'Roll Rate(rad/s)','Yaw Moment(N-m)');
plot(p,Yaw_Moment,'b-','LineWidth',2)
plot(p_Ideal,Yaw_Moment_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

%%%Sideslip SWEEPS
LDM = dlmread('source/Out_Files/SideSlip_Sweeps.FIT');

v = LDM(:,1);
Y = LDM(:,2);
Roll_Moment = LDM(:,3);
Yaw_Moment = LDM(:,4);

%%%Ideal
v_Ideal = linspace(-5,5,100);
V_Ideal = sqrt(Vtrim^2+v_Ideal.^2);
beta_Ideal = asin(v_Ideal./V_Ideal);

Y_Ideal = 0.5*rho*Vtrim^2*S*(C_y_beta.*beta_Ideal);
Roll_Moment_Ideal = 0.5*rho*Vtrim^2*S*b*(C_l_beta.*beta_Ideal);
Yaw_Moment_Ideal = 0.5*rho*Vtrim^2*S*b*(C_n_beta.*beta_Ideal);

plottool(1,'Y(v)',18,'Side Velocity(m/s)','Y(N)');
plot(v,Y,'b-','LineWidth',2)
plot(v_Ideal,Y_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'L(v)',18,'Side Velocity(m/s)','Roll Moment(N-m)');
plot(v,Roll_Moment,'b-','LineWidth',2)
plot(v_Ideal,Roll_Moment_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'N(v)',18,'Side Velocity(m/s)','Yaw Moment(N-m)');
plot(v,Yaw_Moment,'b-','LineWidth',2)
plot(v_Ideal,Yaw_Moment_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

%%%YAW RATE SWEEPS
LDM = dlmread('source/Out_Files/Yaw_Sweeps.FIT');

r = LDM(:,1);
Y = LDM(:,2);
Roll_Moment = LDM(:,3);
Yaw_Moment = LDM(:,4);

%%%Ideal
r_Ideal = linspace(-1,1,100);
r_hat = b.*r_Ideal./(2*Vtrim);
Y_Ideal = 0.5*rho*Vtrim^2*S*(C_y_r.*r_hat);
Roll_Moment_Ideal = 0.5*rho*Vtrim^2*S*b*(C_l_r.*r_hat);
Yaw_Moment_Ideal = 0.5*rho*Vtrim^2*S*b*(C_n_r.*r_hat);

plottool(1,'Y(r)',18,'Yaw Rate(rad/s)','Y(N)');
plot(r,Y,'b-','LineWidth',2)
plot(r_Ideal,Y_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'L(r)',18,'Yaw Rate(rad/s)','Roll Moment(N-m)');
plot(r,Roll_Moment,'b-','LineWidth',2)
plot(r_Ideal,Roll_Moment_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')

plottool(1,'N(r)',18,'Yaw Rate(rad/s)','Yaw Moment(N-m)');
plot(r,Yaw_Moment,'b-','LineWidth',2)
plot(r_Ideal,Yaw_Moment_Ideal,'r-','LineWidth',2)
legend('Lifting Line','Stability Derivative')


