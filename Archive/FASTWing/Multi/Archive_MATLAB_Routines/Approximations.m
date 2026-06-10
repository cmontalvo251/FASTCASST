purge

%%%In order to run this set Mode = 4
%%%Time = 1
%%%open up source code and make sure rfactor = 100

Nmax = 5;
order = Nmax-1; %%Nmax-1
N = 1:Nmax; %%1:Nmax
ANALYTIC = 0;
DEBUG = 0;
MAKEPLOTS = 0;
Ninterp = 100*Nmax; %%100*Nmax

%%%%Plottool 
plottool(1,'All Modes',12,'Real Axis','Imaginary Axis');
slong = [-5.922307 + 9.223556i;-5.922307 + -9.223556i];
roll = [-16.934008 + 0.000000001i;-16.934008 - 0.000000001i];
slat = [-0.418451 + 2.314802i;-0.418451 + -2.314802i];
flong = [-0.032646 + 0.610550i ;-0.032646 + -0.610550i ];
flat = [-0.028812 + 0.0000000001i;-0.028812 + 0.0000000001i];

xlsmatrix = {};

plot(slong,'kx','MarkerSize',10,'LineWidth',2);
plot(roll,'kx','MarkerSize',10,'LineWidth',2);
plot(slat,'kx','MarkerSize',10,'LineWidth',2);
plot(flong,'kx','MarkerSize',10,'LineWidth',2);
plot(flat,'kx','MarkerSize',10,'LineWidth',2);

xlim([-20 5])

xl = xlim;
yl = ylim;

plot([xl(1) xl(2)],[0 0],'k--','LineWidth',1)
plot([0 0],[yl(1) yl(2)],'k--','LineWidth',1)

grid off

%%From Jungs Phd
Along = [-7.1264,0.9497,-0.0485,0.0;-122.8676,-4.292,-0.0261,0.0;4.512,-0.0398,-0.1935,-9.81;0,1,0,0];
Alat = [-0.3301 0.4897 0.0570 -0.9939;0 0 1 0.0570;-35.4688 0 -16.8528 0.1524;2.7315 0 -1.1105 -0.5340];

if ~ANALYTIC
  %system('make rebuild');
  config_data = {'20 20 !Grid_Size_of_Configuration'};
  for ii = 1:20
    config_data = [config_data;num2str(zeros(1,10))];
  end
  config_data = [config_data;'10000 10000 !KLtangent_KLnormal';'40 40 !CLtangent_CLnormal';'1000 !Maximum_Magnet_Force';'372.1041 2584.05625 2584.05625 !KR';'1.4884 10.33623 10.33623 !CR'];
end

[h1,f1] = plottool(1,'Short_Period_Approximation',12,'Real','Imaginary');
[h2,f2] = plottool(1,'Longitudinal Coefficients',12,'Number of Aircraft','Pitch Moment Slope');
[h3,f3] = plottool(1,'Lateral Coefficients',12,'Number of Aircraft','Coefficient');
[h4,f4] = plottool(1,'Geometry',12,'Number of Aircraft','Value(m,m^2,kg*m^2');
[h5,f5] = plottool(1,'Long_Period_Approximation',12,'Real','Imaginary');
[h6,f6] = plottool(1,'Roll Mode',12,'Number of Connected Aircraft',['Real' ...
		    ' Component']);
[h7,f7] = plottool(1,'Dutch_Roll_Approximation',12,'Real','Imaginary');
[h8,f8] = plottool(1,'Roll Mode',12,'Number of Connected Aircraft',['Real' ...
		    ' Component']);

CLa = zeros(length(N),1);
Cmq = zeros(length(N),1);
CLq = zeros(length(N),1);
Cma = zeros(length(N),1);
Cd0 = zeros(length(N),1);
Cm0 = zeros(length(N),1);
CL0 = zeros(length(N),1);
Cda = zeros(length(N),1);
Cybeta = zeros(length(N),1);
Cyp = zeros(length(N),1);
Cyr = zeros(length(N),1);
Cm0 = zeros(length(N),1);
Cma = zeros(length(N),1);
Cmq = zeros(length(N),1);
Clbeta = zeros(length(N),1);
Clp = zeros(length(N),1);
Clr = zeros(length(N),1);
Cnbeta = zeros(length(N),1);
Cnp = zeros(length(N),1);
Cnr = zeros(length(N),1);
Iyy_vec = zeros(length(N),1);
Ixx_vec = zeros(length(N),1);
Ixz_vec = zeros(length(N),1);
Izz_vec = zeros(length(N),1);
S_vec = zeros(length(N),1);
c_vec = zeros(length(N),1);
b_vec = zeros(length(N),1);
W_vec = zeros(length(N),1);

%%%Approximation to different modes
if DEBUG
    S = 185; %ft^2
    b = 33; %ft^2
    c_bar = S/b;
    W = 2800; %lbf
    Vtrim = 180; %ft/s
    Ixx = 1000; %slug-ft^2
    Iyy = 3000; %slug-ft^2
    Izz = 3500; %slug-ft^2
    Ixz = 30; %slug-ft^2
    Ix1 = Ixx;
    Iy1 = Iyy;
    Iz1 = Izz;
    Ixz = 30; %slug-ft^2
    C_D_0 = 0.05;
    C_L_alpha = 4.4;
    C_D_alpha = 0.35;
    C_m_alpha = -0.68;
    C_L_alpha_hat = 1.6;
    C_m_alpha_hat = -4.35;
    C_y_beta = -0.560;
    C_l_beta = -0.0750;
    C_n_beta = 0.070;
    C_D_q = 0;
    C_L_q = 3.8;
    C_m_q = -9.95;
    C_y_p = 0;
    C_l_p = -0.410;
    C_n_p = -0.0575;
    C_y_r = 0.240;
    C_l_r = 0.105;
    C_n_r = -0.125;
    g = 32.17;
    rho = 0.0023769;
    theta0 = 0;
else
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
end

linetype = {'k-','k--'}

for wwtt = 0:1
  
  if wwtt == 0
    W2W = 1;
    T2T = 0;
  else
    T2T = 1;
    W2W = 0;
  end

for ii = 1:length(N)
  
  num_ac = N(ii);

  %%Compute composite c.g and distance vectors from c.g to each aircraft
  rcg2ac = zeros(2,num_ac);
  xyzAC = zeros(2,num_ac);
  idx = 0;
  num_col = 1;
  for idx = 1:num_ac
    if W2W
      xlocation = 0;
      ylocation = (idx-1)*2.04; %%Wing tip to wing tip
    elseif T2T
      xlocation = -(idx-1)*2.04; %%Tip to tail
      ylocation = 0;
    end
    xyzAC(:,idx) = [xlocation;ylocation];
  end
  %%%CG location
  xcg = mean(xyzAC(1,:));
  ycg = mean(xyzAC(2,:));
  CG = [xcg;ycg]*ones(1,num_ac);
  rcg2ac = xyzAC-CG;

  %%% Mass Moment of Inertia Matrix in Body Axis (slug.ft^2)
  grav = 9.81;
  mass1 = 55.07/grav;
  mass = num_ac*mass1;

  I_b1 = [Ix1 0 0;0 Iy1 0;0 0 Iz1];
  %%%Create Parrallel Axis theorem for all of them
  I_b = zeros(3,3);
  for idx = 1:num_ac
    s = [0 0 rcg2ac(2,idx);0 0 -rcg2ac(1,idx);-rcg2ac(2,idx) rcg2ac(1,idx) 0];
    I_cg = I_b1 + s*s'*mass1;
    I_b = I_b + I_cg;
  end
 
  Ixx = I_b(1,1);
  Iyy = I_b(2,2);
  Izz = I_b(3,3);
  Ixz = I_b(3,1);
  Ixx_vec(ii) = Ixx;
  Iyy_vec(ii) = Iyy;
  Izz_vec(ii) = Izz;
  Ixz_vec(ii) = Ixz;
  Izz = I_b(3,3);

  %%Output Important Coefficients and parameters
  if W2W
    c_bar_T = c_bar;
    b_T = num_ac*b;
    S_T = c_bar_T*b_T;
  elseif T2T
    c_bar_T = c_bar*num_ac;
    b_T = b;
    S_T = c_bar_T*b_T;
  end
  W_T = num_ac*W;
  
  W_vec(ii) = W_T;    
  S_vec(ii) = S_T;
  c_vec(ii) = c_bar_T;
  b_vec(ii) = b_T;

  %%Compute C_m_0,C_L_0,C_D_0
  
  if ANALYTIC
    
    %%Zero lift coefficients
    state = [0;0;0;0;0;0;Vtrim;0;0;0;0;0];
    [L_T,M_T,N_T,Lift_T,Drag_T,Y_T] = ComputeFG(state,rcg2ac,num_ac);
    
    C_m_0_T = M_T/(0.5*rho*Vtrim^2*S_T*c_bar_T);
    C_L_0_T = Lift_T/(0.5*rho*Vtrim^2*S_T);
    C_D_0_T = Drag_T/(0.5*rho*Vtrim^2*S_T);
    
    %%Compute C_L_alpha,C_m_alpha,C_D_alpha
    state = [0;0;0;0;0;0;Vtrim;0;1e-4;0;0;0];
    [L_T,M_T,N_T,Lift_T,Drag_T,Y_T] = ComputeFG(state,rcg2ac,num_ac);

    w = state(9);
    u = state(7);
    alpha = atan2(w,u);
    alpha_T = alpha;
    C_m_alpha_T = (M_T/(0.5*rho*Vtrim^2*S_T*c_bar_T) - C_m_0_T)/alpha;
    C_L_alpha_T = (Lift_T/(0.5*rho*Vtrim^2*S_T) - C_L_0_T)/alpha;
    C_D_alpha_T = (Drag_T/(0.5*rho*Vtrim^2*S_T) - C_D_0_T)/alpha^2;
  
    %%%Compute C_m_q and C_L_q
    state = [0;0;0;0;0;0;Vtrim;0;0;0;1e-8;0];
    [L_T,M_T,N_T,Lift_T,Drag_T,Y_T] = ComputeFG(state,rcg2ac,num_ac);

    q = state(11);
    qhat = c_bar_T * q / (2 * Vtrim);
    C_m_q_T = (M_T/(0.5*rho*Vtrim^2*S_T*c_bar_T) - C_m_0_T)/qhat;
    C_L_q_T = (Lift_T/(0.5*rho*Vtrim^2*S_T)-C_L_0_T)/qhat;
    
    %%%Compute C_l_p, C_n_p, and C_y_p
    state = [0;0;0;0;0;0;Vtrim;0;0;1e-8;0;0];    
    [L_T,M_T,N_T,Lift_T,Drag_T,Y_T] = ComputeFG(state,rcg2ac,num_ac);    
     
    p = state(10);
    phat = b_T*p/(2*Vtrim);
    C_l_p_T = (L_T/(0.5*rho*Vtrim^2*S_T*b_T))/phat;
    C_n_p_T = (N_T/(0.5*rho*Vtrim^2*S_T*b_T))/phat;
    C_y_p_T = (Y_T/(0.5*rho*Vtrim^2*S_T))/phat;
    
    %%%Compute C_l_beta, C_n_beta, and C_y_beta
    state = [0;0;0;0;0;0;Vtrim;1e-8;0;0;0;0];    
    [L_T,M_T,N_T,Lift_T,Drag_T,Y_T] = ComputeFG(state,rcg2ac,num_ac);    
     
    v = state(8);
    beta = asin(v/Vtrim);
    C_l_beta_T = (L_T/(0.5*rho*Vtrim^2*S_T*b_T))/beta;
    C_n_beta_T = (N_T/(0.5*rho*Vtrim^2*S_T*b_T))/beta;
    C_y_beta_T = (Y_T/(0.5*rho*Vtrim^2*S_T))/beta;
    
    %%%Compute C_n_r, C_l_r, and C_y_r
    state = [0;0;0;0;0;0;Vtrim;0;0;0;0;1e-8];    
    [L_T,M_T,N_T,Lift_T,Drag_T,Y_T] = ComputeFG(state,rcg2ac,num_ac);    
     
    r = state(12);
    rhat = b_T*r/(2*Vtrim);
    C_n_r_T = (N_T/(0.5*rho*Vtrim^2*S_T*b_T))/rhat;
    C_l_r_T = (L_T/(0.5*rho*Vtrim^2*S_T*b_T))/rhat;
    C_y_r_T = (Y_T/(0.5*rho*Vtrim^2*S_T))/rhat;

  else
    %%Numerically obtain it
    writematrix = zeros(20,20);
    configuration = ones(num_ac,1);
    if W2W
      writematrix(1,1:num_ac) = configuration';
    else
      writematrix(1:num_ac,1) = configuration;
    end
    for idx = 1:20
      config_data{idx+1} = num2str(writematrix(idx,:));
    end
    writedata('source/Input_Files/Configuration.CONN',config_data);
    %%%RUNCODE HERE
    system('./Run.exe source/MultiMeta.ifiles');
    FM = dlmread('source/Out_Files/Meta_Coefficients.OUT');
    
    %%%ANGLE OF ATTACK SWEEPS
    LDM = dlmread('source/Out_Files/AOA_Sweeps.FIT');

    alfa = LDM(:,1).*180/pi;
    alfa_rad = alfa.*pi/180;
    Lift = LDM(:,2);
    Drag = LDM(:,3);
    Pitch_Moment = LDM(:,4);
    Qa = (0.5*rho*Vtrim^2*S_T);
    
    L0 = interp1(alfa,Lift,0);
    D0 = interp1(alfa,Drag,0);
    M0 = interp1(alfa,Pitch_Moment,0);
    
    C_L_0_T = L0/Qa;
    C_D_0_T = D0/Qa;
    C_m_0_T = M0/(Qa*c_bar_T);
    
    alfai = 4;
    
    L5 = interp1(alfa,Lift,alfai);
    D5 = interp1(alfa,Drag,alfai);
    M5 = interp1(alfa,Pitch_Moment,alfai);
    
    alfai = alfai*pi/180;
    
    C_L_alpha_T = (L5/Qa-C_L_0_T)/alfai;
    C_D_alpha_T = (D5/Qa-C_D_0_T)/alfai^2;
    C_m_alpha_T = (M5/(Qa*c_bar_T)-C_m_0_T)/alfai;

    if MAKEPLOTS
      plottool(1,'L(AOA)',18,'Angle of Attack(deg)','Lift(N)');
      plot(alfa,Lift,'b-','LineWidth',2)
      Lift_T = Qa*(C_L_0_T+C_L_alpha_T.*alfa_rad);
      plot(alfa,Lift_T,'r-','LineWidth',2)

      plottool(1,'D(AOA)',18,'Angle of Attack(deg)','Drag(N)');
      Drag_T = Qa*(C_D_0_T + C_D_alpha_T.*alfa_rad.^2);
      plot(alfa,Drag,'b-','LineWidth',2)
      plot(alfa,Drag_T,'r-','LineWidth',2)
    
      plottool(1,'M(AOA)',18,'Angle of Attack(deg)','Pitch Moment(N)');
      Pitch_Moment_T = Qa*c_bar_T*(C_m_0_T+C_m_alpha_T.*alfa_rad);
      plot(alfa,Pitch_Moment,'b-','LineWidth',2)
      plot(alfa,Pitch_Moment_T,'r-','LineWidth',2)
    end
    
    %%%PITCH RATE SWEEPS
    LDM = dlmread('source/Out_Files/Pitch_Sweeps.FIT');

    q = LDM(:,1);
    Lift = LDM(:,2);
    Pitch_Moment = LDM(:,3);

    qi = 0.25;
    L5 = interp1(q,Lift,qi);
    M5 = interp1(q,Pitch_Moment,qi);
    qhat = qi*c_bar_T/(2*Vtrim);
    C_L_q_T = ((L5/Qa)-C_L_0_T)/qhat;
    C_m_q_T = ((M5/(Qa*c_bar_T))-C_m_0_T)/qhat;
    
    q_Fit = q;
    q_Fit_hat = q_Fit.*c_bar_T/(2*Vtrim);
    Pitch_Moment_Fit = Qa*c_bar_T*(C_m_0_T + C_m_q_T.*q_Fit_hat);

    if MAKEPLOTS
      plottool(1,'L(q)',18,'Pitch Rate(rad/s)','Lift(N)');
      plot(q,Lift,'b-','LineWidth',2)
      Lift_q_T = Qa.*(C_L_q_T.*q_Fit_hat+C_L_0_T);
      plot(q,Lift_q_T,'r-','LineWidth',2)

      plottool(1,'M(q)',18,'Pitch Rate(rad/s)','Pitch Moment(N)');
      plot(q,Pitch_Moment,'b-','LineWidth',2)
      plot(q_Fit,Pitch_Moment_Fit,'r-','LineWidth',2)
    end

    %%%ROLL RATE SWEEPS
    LDM = dlmread('source/Out_Files/Roll_Sweeps.FIT');

    p = LDM(:,1);
    Y = LDM(:,2);
    Roll_Moment = LDM(:,3);
    Yaw_Moment = LDM(:,4);
    
    pii = 0.25;
    phat = b_T*pii/(2*Vtrim);
    p_Fit_hat = b_T*p/(2*Vtrim);
    Y5 = interp1(p,Y,pii);
    L5 = interp1(p,Roll_Moment,pii);
    N5 = interp1(p,Yaw_Moment,pii);
    
    C_y_p_T = Y5/(Qa*phat);
    C_l_p_T = L5/(Qa*b_T*phat);
    C_n_p_T = N5/(Qa*b_T*phat);

    if MAKEPLOTS
      plottool(1,'Y(p)',18,'Roll Rate(rad/s)','Y(N)');
      plot(p,Y,'b-','LineWidth',2)
      Y_T = Qa*p_Fit_hat*C_y_p_T;
      plot(p,Y_T,'r-','LineWidth',2)

      plottool(1,'L(p)',18,'Roll Rate(rad/s)','Roll Moment(N-m)');
      plot(p,Roll_Moment,'b-','LineWidth',2)
      Roll_Moment_T = C_l_p_T*Qa*b_T*p_Fit_hat;
      plot(p,Roll_Moment_T,'r-','LineWidth',2)
      
      plottool(1,'N(p)',18,'Roll Rate(rad/s)','Yaw Moment(N-m)');
      Yaw_Moment_T = C_n_p_T*Qa*b_T*p_Fit_hat;
      plot(p,Yaw_Moment,'b-','LineWidth',2)
      plot(p,Yaw_Moment_T,'r-','LineWidth',2)
    end
    

    %%%Sideslip SWEEPS
    LDM = dlmread('source/Out_Files/SideSlip_Sweeps.FIT');

    v = LDM(:,1);
    Y = LDM(:,2);
    Roll_Moment = LDM(:,3);
    Yaw_Moment = LDM(:,4);
    
    vi = 0.25;
    beta = asin(vi/Vtrim);
    beta_Fit = asin(v./Vtrim);
    Y5 = interp1(v,Y,vi);
    L5 = interp1(v,Roll_Moment,vi);
    N5 = interp1(v,Yaw_Moment,vi);

    C_y_beta_T = Y5/(Qa*beta);
    C_l_beta_T = L5/(Qa*b_T*beta);
    C_n_beta_T = N5/(Qa*b_T*beta);

    if MAKEPLOTS
      plottool(1,'Y(v)',18,'Side Velocity(m/s)','Y(N)');
      plot(v,Y,'b-','LineWidth',2)
      Y_v_T = C_y_beta_T*Qa*beta_Fit;
      plot(v,Y_v_T,'r-','LineWidth',2)

      plottool(1,'L(v)',18,'Side Velocity(m/s)','Roll Moment(N-m)');
      plot(v,Roll_Moment,'b-','LineWidth',2)
      Roll_Moment_T = C_l_beta_T*Qa*b_T*beta_Fit;
      plot(v,Roll_Moment_T,'r-','LineWidth',2)

      plottool(1,'N(v)',18,'Side Velocity(m/s)','Yaw Moment(N-m)');
      plot(v,Yaw_Moment,'b-','LineWidth',2)
      Yaw_Moment_T = C_n_beta_T*Qa*b_T*beta_Fit;
      plot(v,Yaw_Moment_T,'r-','LineWidth',2)
    end
    
    %%%YAW RATE SWEEPS
    LDM = dlmread('source/Out_Files/Yaw_Sweeps.FIT');

    r = LDM(:,1);
    Y = LDM(:,2);
    Roll_Moment = LDM(:,3);
    Yaw_Moment = LDM(:,4);
    
    rii = 0.25;
    rhat = b_T*rii/(2*Vtrim);
    r_hat_Fit = b_T*r/(2*Vtrim);
    Y5 = interp1(r,Y,rii);
    L5 = interp1(r,Roll_Moment,rii);
    N5 = interp1(r,Yaw_Moment,rii);
    
    C_y_r_T = Y5/(Qa*rhat);
    C_l_r_T = L5/(Qa*b_T*rhat);
    C_n_r_T = N5/(Qa*b_T*rhat);

    if MAKEPLOTS
      plottool(1,'Y(r)',18,'Yaw Rate(rad/s)','Y(N)');
      plot(r,Y,'b-','LineWidth',2)
      Y_T = C_y_r_T*Qa*r_hat_Fit;
      plot(r,Y_T,'r-','LineWidth',2)
      
      plottool(1,'L(r)',18,'Yaw Rate(rad/s)','Roll Moment(N-m)');
      plot(r,Roll_Moment,'b-','LineWidth',2)
      Roll_Moment_T = C_l_r_T*Qa*b_T*r_hat_Fit;
      plot(r,Roll_Moment_T,'r-','LineWidth',2)
      
      plottool(1,'N(r)',18,'Yaw Rate(rad/s)','Yaw Moment(N-m)');
      plot(r,Yaw_Moment,'b-','LineWidth',2)
      Yaw_Moment_T = C_n_r_T*Qa*b_T*r_hat_Fit;
      plot(r,Yaw_Moment_T,'r-','LineWidth',2)
    end
    
  end
  
  if DEBUG
    C_L_0_T = W_T*cos(theta0)/(0.5*rho*Vtrim^2*S_T);
    C_m_0_T = 0;
    C_D_0_T = C_D_0;
    C_m_alpha_T = C_m_alpha;
    C_L_alpha_T = C_L_alpha;
    C_D_alpha_T = C_D_alpha;
    C_m_q_T = C_m_q;
    C_L_q_T = C_L_q;
    C_l_p_T = C_l_p;
    C_l_beta_T = C_l_beta;
    C_y_beta_T = C_y_beta;
    C_y_p_T = C_y_p;
    C_y_r_T = C_y_r;
    C_l_r_T = C_l_r;
    C_n_p_T = C_n_p;
    C_n_r_T = C_n_r;
    C_n_beta_T = C_n_beta;
  end
  
  CL0(ii) = C_L_0_T;
  CLa(ii) = C_L_alpha_T;
  CLq(ii) = C_L_q_T;
  
  Cd0(ii) = C_D_0_T;
  Cda(ii) = C_D_alpha_T;
  
  Cm0(ii) = C_m_0_T;
  Cma(ii) = C_m_alpha_T;
  Cmq(ii) = C_m_q_T;
  
  Clbeta(ii) = C_l_beta_T;
  Clp(ii) = C_l_p_T;
  Clr(ii) = C_l_r_T;
  
  Cnbeta(ii) = C_n_beta_T;
  Cnp(ii) = C_n_p_T;
  Cnr(ii) = C_n_r_T;
  
  Cybeta(ii) = C_y_beta_T;
  Cyp(ii) = C_y_p_T;
  Cyr(ii) = C_y_r_T;

end

%%%Plot Longitudinal Coefficients
%plot(f2,N,CLa,'k*','LineWidth',2,'MarkerSize',10)
%plot(f2,N,Cmq,'kv','LineWidth',2,'MarkerSize',10)
%plot(f2,N,CLq,'k>','LineWidth',2,'MarkerSize',10)
%plot(f2,N,Cma,'ks','LineWidth',2,'MarkerSize',10)
%plot(f2,N,Cd0,'kx','LineWidth',2,'MarkerSize',10)
%plot(f2,N,Cda,'ko','LineWidth',2,'MarkerSize',10)
%plot(f2,N,CL0,'kd','LineWidth',2,'MarkerSize',10)
%plot(f2,N,Cm0,'k^','LineWidth',2,'MarkerSize',10)
%legend(f2,'CLalpha','Cmq','CLq','Cmalpha','Cd0','Cda','CL0','Cm0')

%%%Plot Longitudinal Coefficients
plot(f3,N,Clbeta,'k*','LineWidth',2,'MarkerSize',10)
plot(f3,N,Clp,'kv','LineWidth',2,'MarkerSize',10)
plot(f3,N,Clr,'k>','LineWidth',2,'MarkerSize',10)
plot(f3,N,Cnbeta,'ks','LineWidth',2,'MarkerSize',10)
plot(f3,N,Cnp,'kx','LineWidth',2,'MarkerSize',10)
plot(f3,N,Cnr,'ko','LineWidth',2,'MarkerSize',10)
plot(f3,N,Cybeta,'kd','LineWidth',2,'MarkerSize',10)
plot(f3,N,Cyp,'k^','LineWidth',2,'MarkerSize',10)
plot(f3,N,Cyr,'k<','LineWidth',2,'MarkerSize',10)
legend(f3,'Clbeta','Clp','Clr','Cnbeta','Cnp','Cnr','Cybeta','Cyp','Cyr')

%%%%Plot Geometric stuff
plot(f4,N,Ixx_vec,'ks','LineWidth',2,'MarkerSize',10)
plot(f4,N,Iyy_vec,'ks','LineWidth',2,'MarkerSize',10)
plot(f4,N,Izz_vec,'ks','LineWidth',2,'MarkerSize',10)
plot(f4,N,S_vec,'ko','LineWidth',2,'MarkerSize',10)
plot(f4,N,b_vec,'k*','LineWidth',2,'MarkerSize',10)
plot(f4,N,c_vec,'kx','LineWidth',2,'MarkerSize',10)
plot(f4,N,W_vec,'k^','LineWidth',2,'MarkerSize',10)
%legend(f3,'Iyy(kg-m^2)','S(m^2)','c(m)','W(N)')
legend(f4,'Ixx(kg-m^2)','Iyy(kg-m^2)','Izz(kg-m^2)','S(m^2)','b(m)','c(m)','W(N)')

%%%%Create Polynomial fits for all coefficients
Nc = linspace(N(1),N(end),Ninterp);

%%%%Interpolate Longitudinal Coefficients
pcla = polyfit(N',CLa,order);    
CLafit = polyval(pcla,Nc);
%plot(f2,Nc,CLafit,linetype{wwtt+1},'LineWidth',2)

pcmq = polyfit(N',Cmq,order);    
Cmqfit = polyval(pcmq,Nc);
%plot(f2,Nc,Cmqfit,linetype{wwtt+1},'LineWidth',2)

pclq = polyfit(N',CLq,order);    
CLqfit = polyval(pclq,Nc);
%plot(f2,Nc,CLqfit,linetype{wwtt+1},'LineWidth',2)

pcma = polyfit(N',Cma,order);    
Cmafit = polyval(pcma,Nc);
plot(f2,Nc,Cmafit,linetype{wwtt+1},'LineWidth',2)

pcd0 = polyfit(N',Cd0,order);    
Cd0fit = polyval(pcd0,Nc);
%plot(f2,Nc,Cd0fit,linetype{wwtt+1},'LineWidth',2)

pcl0 = polyfit(N',CL0,order);    
CL0fit = polyval(pcl0,Nc);
%plot(f2,Nc,CL0fit,linetype{wwtt+1},'LineWidth',2)

pcm0 = polyfit(N',Cm0,order);    
Cm0fit = polyval(pcm0,Nc);
%plot(f2,Nc,Cm0fit,linetype{wwtt+1},'LineWidth',2)

pcda = polyfit(N',Cda,order);    
Cdafit = polyval(pcda,Nc);
%plot(f2,Nc,Cdafit,linetype{wwtt+1},'LineWidth',2)

%%%%Interpolate Lateral Coefficients
pcybeta = polyfit(N',Cybeta,order);    
cybetafit = polyval(pcybeta,Nc);
plot(f3,Nc,cybetafit,linetype{wwtt+1},'LineWidth',2)

pcyp = polyfit(N',Cyp,order);    
cypfit = polyval(pcyp,Nc);
plot(f3,Nc,cypfit,linetype{wwtt+1},'LineWidth',2)

pcyr = polyfit(N',Cyr,order);    
cyrfit = polyval(pcyr,Nc);
plot(f3,Nc,cyrfit,linetype{wwtt+1},'LineWidth',2)

pclbeta = polyfit(N',Clbeta,order);    
clbetafit = polyval(pclbeta,Nc);
plot(f3,Nc,clbetafit,linetype{wwtt+1},'LineWidth',2)

pclp = polyfit(N',Clp,order);    
clpfit = polyval(pclp,Nc);
plot(f3,Nc,clpfit,linetype{wwtt+1},'LineWidth',2)

pclr = polyfit(N',Clr,order);    
clrfit = polyval(pclr,Nc);
plot(f3,Nc,clrfit,linetype{wwtt+1},'LineWidth',2)

pcnbeta = polyfit(N',Cnbeta,order);    
cnbetafit = polyval(pcnbeta,Nc);
plot(f3,Nc,cnbetafit,linetype{wwtt+1},'LineWidth',2)

pcnp = polyfit(N',Cnp,order);    
cnpfit = polyval(pcnp,Nc);
plot(f3,Nc,cnpfit,linetype{wwtt+1},'LineWidth',2)

pcnr = polyfit(N',Cnr,order);    
cnrfit = polyval(pcnr,Nc);
plot(f3,Nc,cnrfit,linetype{wwtt+1},'LineWidth',2)

%%%%%Interpolate Geometric Properties
pIyy = polyfit(N',Iyy_vec,order);    
Iyyfit = polyval(pIyy,Nc);
plot(f4,Nc,Iyyfit,linetype{wwtt+1},'LineWidth',2)

pIxx = polyfit(N',Ixx_vec,order);    
Ixxfit = polyval(pIxx,Nc);
plot(f4,Nc,Ixxfit,linetype{wwtt+1},'LineWidth',2)

pIxz = polyfit(N',Ixz_vec,order);    
Ixzfit = polyval(pIxz,Nc);

pIzz = polyfit(N',Izz_vec,order);    
Izzfit = polyval(pIzz,Nc);
plot(f4,Nc,Izzfit,linetype{wwtt+1},'LineWidth',2)

pS = polyfit(N',S_vec,order);    
Sfit = polyval(pS,Nc);
plot(f4,Nc,Sfit,linetype{wwtt+1},'LineWidth',2)

pc = polyfit(N',c_vec,order);    
cfit = polyval(pc,Nc);
plot(f4,Nc,cfit,linetype{wwtt+1},'LineWidth',2)

pw = polyfit(N',W_vec,order);    
wfit = polyval(pw,Nc);
plot(f4,Nc,wfit,linetype{wwtt+1},'LineWidth',2)

pb = polyfit(N',b_vec,order);
bfit = polyval(pb,Nc);
plot(f4,Nc,bfit,linetype{wwtt+1},'LineWidth',2)

real_lambda1_vec = zeros(length(Nc),1);
imag_lambda1_vec = zeros(length(Nc),1);
real_lambda2_vec = zeros(length(Nc),1);
imag_lambda2_vec = zeros(length(Nc),1);

real_lambdalp1_vec = zeros(length(Nc),1);
imag_lambdalp1_vec = zeros(length(Nc),1);
real_lambdalp2_vec = zeros(length(Nc),1);
imag_lambdalp2_vec = zeros(length(Nc),1);

roll_vec = zeros(length(Nc),1);
roll_Avec = zeros(length(Nc),1);

spiral_vec = zeros(length(Nc),1);
spiral_Avec = zeros(length(Nc),1);

real_lambdafl1_vec = zeros(length(Nc),1);
imag_lambdafl1_vec = zeros(length(Nc),1);
real_lambdafl2_vec = zeros(length(Nc),1);
imag_lambdaft2_vec = zeros(length(Nc),1);

real_lambda1_Avec = zeros(length(Nc),1);
imag_lambda1_Avec = zeros(length(Nc),1);
real_lambda2_Avec = zeros(length(Nc),1);
imag_lambda2_Avec = zeros(length(Nc),1);

real_lambdalp1_Avec = zeros(length(Nc),1);
imag_lambdalp1_Avec = zeros(length(Nc),1);
real_lambdalp2_Avec = zeros(length(Nc),1);
imag_lambdalp2_Avec = zeros(length(Nc),1);

real_lambdafl1_Avec = zeros(length(Nc),1);
imag_lambdafl1_Avec = zeros(length(Nc),1);
real_lambdafl2_Avec = zeros(length(Nc),1);
imag_lambdaft2_Avec = zeros(length(Nc),1);

for ii = 1:length(Nc)
  disp(['Number of Aircraft = ',num2str(ii)])
  b_T = bfit(ii);
  c_bar_T = cfit(ii);
  S_T = Sfit(ii);
  W_T = wfit(ii);
  Iyy = Iyyfit(ii);
  Ixx = Ixxfit(ii);
  Ixz = Ixzfit(ii);
  Izz = Izzfit(ii);
  C_L_alpha_T = CLafit(ii);
  C_m_q_T = Cmqfit(ii);
  C_L_q_T = CLqfit(ii);
  C_m_alpha_T = Cmafit(ii);
  C_D_0_T = Cd0fit(ii);
  C_D_alpha_T = Cdafit(ii);
  C_m_0_T = Cm0fit(ii);
  C_L_0_T = CL0fit(ii);
  C_y_beta_T = cybetafit(ii);
  C_y_p_T = cypfit(ii);
  C_y_r_T = cyrfit(ii);
  C_l_beta_T = clbetafit(ii);
  C_l_p_T = clpfit(ii);
  C_l_r_T = clrfit(ii);
  C_n_beta_T = cnbetafit(ii);
  C_n_p_T = cnpfit(ii);
  C_n_r_T = cnrfit(ii);
  
  %%Longitudinal Approximation
  Rgx = g*c_bar_T/(2*Vtrim^2);
  R_z_alpha_hat = rho*S_T*c_bar_T/(4*(W_T/g))*(-C_L_alpha_hat);
  R_m_alpha_hat = rho*S_T*(c_bar_T^3)/(8*Iyy)*C_m_alpha_hat;
  R_x_mu = -rho*S_T*c_bar_T/(4*(W_T/g))*2*C_D_0_T;
  theta0 = 0;
  C_L_o = W_T*cos(theta0)/(0.5*rho*Vtrim^2*S_T);
  R_z_mu = -rho*S_T*c_bar_T/(4*(W_T/g))*2*C_L_o;
  R_m_mu = 0;
  %%%Compute C_D_alpha properly for stability derivative
  H = [1 0 ;1 10*pi/180];
  X = [0 C_D_alpha_T*(10*pi/180)^2]';
  thetastar = inv(H'*H)*H'*X;
  C_D_alpha_1 = thetastar(2);
  R_x_alpha = rho*S_T*c_bar_T/(4*(W_T/g))*(C_L_o-C_D_alpha_1);
  R_z_alpha = rho*S_T*c_bar_T/(4*(W_T/g))*(-C_L_alpha_T-C_D_0_T);
  R_m_alpha = rho*S_T*c_bar_T^3/(8*Iyy)*C_m_alpha_T;
  R_x_q = rho*S_T*c_bar_T/(4*(W_T/g))*-C_D_q;
  R_z_q = rho*S_T*c_bar_T/(4*(W_T/g))*-C_L_q_T; 
  R_m_q = rho*S_T*c_bar_T^3/(8*Iyy)*C_m_q_T;

  %%%Short Period Approximation
  %%%Non-dimensional
  disp('Short Period')
  lambda_nd1 = (R_z_alpha+R_m_q+R_m_alpha_hat)/2 + sqrt(((R_z_alpha+R_m_q+R_m_alpha_hat)/2)^2-(R_z_alpha*R_m_q-R_m_alpha));
  lambda_nd2 = (R_z_alpha+R_m_q+R_m_alpha_hat)/2 - sqrt(((R_z_alpha+R_m_q+R_m_alpha_hat)/2)^2-(R_z_alpha*R_m_q-R_m_alpha));
  lambda1 = 2*Vtrim/c_bar_T*lambda_nd1
  lambda2 = 2*Vtrim/c_bar_T*lambda_nd2
  
  real_lambda1_vec(ii) = real(lambda1);
  imag_lambda1_vec(ii) = imag(lambda1);
  real_lambda2_vec(ii) = real(lambda2);
  imag_lambda2_vec(ii) = imag(lambda2);

  %%%Long Period Approximation
  Rs = R_m_alpha/(R_m_alpha-R_z_alpha*R_m_q);
  Rd = R_x_alpha*R_m_q/(R_m_alpha-R_z_alpha*R_m_q);
  Rp = Rgx*Rs*(R_z_alpha+R_m_q)/(R_m_alpha-R_z_alpha*R_m_q);
  %C_D = C_D_0_T + C_D_alpha_1*alpha_T^2;
  %C_L = C_L_0_T + C_L_alpha_1*alpha_T;
  disp('Long Period')
  %%%Eqn 8.4.8 and 8.4.13
  %lambda_nd1 = R_x_mu/2 + sqrt((R_x_mu/2)^2+Rgx*R_z_mu);
  %lambda_nd2 = R_x_mu/2 + sqrt((R_x_mu/2)^2+Rgx*R_z_mu);
  %lambda_nd1 = (g*c_bar_T/(2*Vtrim^2))*(-C_D_0_T/C_L_0_T+i*sqrt(2-(C_D_0_T/C_L_0_T)^2));
  %lambda_nd2 = (g*c_bar_T/(2*Vtrim^2))*(-C_D_0_T/C_L_0_T+i*sqrt(2-(C_D_0_T/C_L_0_T)^2));
  %%%Eqn 8.4.44 and Eqn 8.4.52  
  lambda_nd1 = (R_z_mu/2)*((R_x_mu/R_z_mu+Rd-Rp)-i*sqrt(-4*Rgx/R_z_mu*Rs-(R_x_mu/R_z_mu+Rd)^2));
  lambda_nd2 = (R_z_mu/2)*((R_x_mu/R_z_mu+Rd-Rp)+i*sqrt(-4*Rgx/R_z_mu*Rs-(R_x_mu/R_z_mu+Rd)^2));
  %lambda_nd1 = g*c_bar_T/(2*Vtrim^2)*((-C_D_0_T/C_L_0_T-Rd+Rp)+i*sqrt(2*Rs-(C_D_0_T/C_L_0_T+Rd)^2));
  %lambda_nd2 = g*c_bar_T/(2*Vtrim^2)*((-C_D_0_T/C_L_0_T-Rd+Rp)-i*sqrt(2*Rs-(C_D_0_T/C_L_0_T+Rd)^2));

  lambda1 = 2*Vtrim/c_bar_T*lambda_nd1
  lambda2 = 2*Vtrim/c_bar_T*lambda_nd2
  
  real_lambdalp1_vec(ii) = real(lambda1);
  imag_lambdalp1_vec(ii) = imag(lambda1);
  real_lambdalp2_vec(ii) = real(lambda2);
  imag_lambdalp2_vec(ii) = imag(lambda2);
  
  %%%Longintudinal A matrix
  A = zeros(6,6);
  A(1,1) = R_x_mu;
  A(1,2) = R_x_alpha;
  A(1,3) = R_x_q;
  A(1,6) = -Rgx*cos(theta0);
  A(2,1) = R_z_mu/(1-R_z_alpha_hat);
  A(2,2) = R_z_alpha/(1-R_z_alpha_hat);
  A(2,3) = (1+R_z_q)/(1-R_z_alpha_hat);
  A(2,6) = -Rgx*sin(theta0)/(1-R_z_alpha_hat);
  A(3,1) = R_m_mu+R_m_alpha_hat*R_z_mu/(1-R_z_alpha_hat);
  A(3,2) = R_m_alpha+R_m_alpha_hat*R_z_alpha/(1-R_z_alpha_hat);
  A(3,3) = R_m_q+R_m_alpha_hat*(1+R_z_q)/(1-R_z_alpha_hat);
  A(3,6) = -R_m_alpha_hat*Rgx*sin(theta0)/(1-R_z_alpha_hat);
  A(4,1) = cos(theta0);
  A(4,2) = sin(theta0);
  A(4,6) = -sin(theta0);
  A(5,1) = -sin(theta0);
  A(5,2) = cos(theta0);
  A(5,6) = -cos(theta0);
  A(6,3) = 1;
  
  long_modes_nd = eig(A);
  A_long = A;
  
  long_modes = 2*Vtrim/(c_bar_T)*long_modes_nd
  
  real_lambda1_Avec(ii) = real(long_modes(3));
  imag_lambda1_Avec(ii) = imag(long_modes(3));
  real_lambda2_Avec(ii) = real(long_modes(4));
  imag_lambda2_Avec(ii) = imag(long_modes(4));
  
  real_lambdalp1_Avec(ii) = real(long_modes(5));
  imag_lambdalp1_Avec(ii) = imag(long_modes(5));
  real_lambdalp2_Avec(ii) = real(long_modes(6));
  imag_lambdalp2_Avec(ii) = imag(long_modes(6));
  
  %%%Jungs Phd Thesis
  long_modes_jung = eig(Along);
  
  %%%Roll Mode
  R_l_p = (rho*S_T*b_T^3/(8*Ixx))*C_l_p_T;
  disp('Roll mode')
  %%%Eqn 9.3.3
  lambda_nd = R_l_p;
  lambda = (2*Vtrim/b_T)*lambda_nd
  
  roll_vec(ii) = lambda;
  
  %%%Spiral Mode
  disp('Spiral Mode')
  lambda_nd = -g*b_T/(2*Vtrim^2)*((C_l_beta_T*C_n_r_T-C_l_r_T*C_n_beta_T)/(C_l_beta_T*C_n_p_T-C_l_p_T*C_n_beta_T));
  lambda = 2*Vtrim/b_T*lambda_nd
  
  spiral_vec(ii) = lambda;
  
  %%%Dutch Roll Mode
  disp('Dutch Roll Mode')
  fw = rho*S_T*b_T/(4*W_T/g);
  fxx = rho*S_T*b_T^3/(8*Ixx);
  fzz = rho*S_T*b_T^3/(8*Izz);
  R_y_beta = fw*C_y_beta_T;
  R_l_beta = fxx*C_l_beta_T;
  R_n_beta = fzz*C_n_beta_T;
  R_y_p = fw*C_y_p_T;
  R_l_p = fxx*C_l_p_T;
  R_n_p = fzz*C_n_p_T;
  R_y_r = fw*C_y_r_T;
  R_l_r = fxx*C_l_r_T;
  R_n_r = fzz*C_n_r_T;
  Rgy = g*b_T/(2*Vtrim^2);
  R_Ds = (R_l_beta*(Rgy-(1-R_y_r)*R_n_p)-R_y_beta*R_l_r*R_n_p)/R_l_p;
  R_Dc = R_l_r*R_n_p/R_l_p;
  R_Dp = Rgy*(R_l_r*R_n_beta-R_l_beta*R_n_r)/(R_l_p*(R_n_beta+R_y_beta*R_n_r))-R_Ds/R_l_p;
  lambda_nd1 = (R_y_beta+R_n_r-R_Dc+R_Dp)/2 + i*sqrt((1-R_y_r)*R_n_beta+R_y_beta*R_n_r+R_Ds-((R_y_beta+R_n_r)/2)^2);
  lambda_nd2 = (R_y_beta+R_n_r-R_Dc+R_Dp)/2 - i*sqrt((1-R_y_r)*R_n_beta+R_y_beta*R_n_r+R_Ds-((R_y_beta+R_n_r)/2)^2);
  
  lambda1 = (2*Vtrim/b_T)*lambda_nd1
  lambda2 = (2*Vtrim/b_T)*lambda_nd2
  
  real_lambdafl1_vec(ii) = real(lambda1);
  imag_lambdafl1_vec(ii) = imag(lambda1);
  real_lambdafl2_vec(ii) = real(lambda2);
  imag_lambdafl2_vec(ii) = imag(lambda2);
  
  %%%Lateral A matrix
  lxz = Ixz/Ixx;
  lzx = Ixz/Izz;
  A = zeros(6,6);
  A(1,1) = R_y_beta;
  A(1,2) = R_y_p;
  A(1,3) = R_y_r-1;
  A(1,5) = Rgy*cos(theta0);
  A(2,1) = (R_l_beta+lxz*R_n_beta)/(1-lxz*lzx);
  A(2,2) = (R_l_p+lxz*R_n_p)/(1-lxz*lzx);
  A(2,3) = (R_l_r+lxz*R_n_r)/(1-lxz*lzx);
  A(3,1) = (R_n_beta+lzx*R_l_beta)/(1-lxz*lzx);
  A(3,2) = (R_n_p+lzx*R_l_p)/(1-lxz*lzx);
  A(3,3) = (R_n_r+lzx*R_l_r)/(1-lxz*lzx);
  A(4,1) = 1;
  A(4,6) = cos(theta0);
  A(5,2) = 1;
  A(5,3) = tan(theta0);
  A(6,3) = sec(theta0);
  A_lat = A;
  
  lat_modes_nd = eig(A);
  
  lat_modes = lat_modes_nd*2*Vtrim/b_T

  if ii > 1
    locroll = find(abs(roll_Avec(ii-1)-lat_modes) == min(abs(roll_Avec(ii-1)-lat_modes)));
  else
    locroll = 3;
  end
  roll_Avec(ii) = lat_modes(locroll);
  
  if ii > 1
    lat_modes_nz = lat_modes(3:end);
    locsp = find(abs(spiral_Avec(ii-1)-lat_modes_nz) == min(abs(spiral_Avec(ii-1)-lat_modes_nz)));
    locsp = locsp + 2;
  else
    locsp = 6;
  end
  spiral_Avec(ii) = lat_modes(locsp);
  
  lat_modes([locroll,locsp]) = [];
  
  real_lambdafl1_Avec(ii) = real(lat_modes(3));
  imag_lambdafl1_Avec(ii) = imag(lat_modes(3));
  real_lambdafl2_Avec(ii) = real(lat_modes(4));
  imag_lambdafl2_Avec(ii) = imag(lat_modes(4));
  
  %%%Jungs Phd Thesis
  lat_modes_jung = eig(Alat);
    
end
%clc
%close all
fprintf('CL0 %f %f \n',C_L_0,C_L_0_T)
fprintf('Cm0 %f %f \n',C_m_0,C_m_0_T)
fprintf('CD0 %f %f \n',C_D_0,C_D_0_T)
fprintf('CLa %f %f \n',C_L_alpha,C_L_alpha_T)
fprintf('CDa %f %f \n',C_D_alpha,C_D_alpha_T)
fprintf('Cma %f %f \n',C_m_alpha,C_m_alpha_T)
fprintf('Cmq %f %f \n',C_m_q,C_m_q_T)
fprintf('CLq %f %f \n',C_L_q,C_L_q_T)
fprintf('Clp %f %f \n',C_l_p,C_l_p_T)
fprintf('Cnp %f %f \n',C_n_p,C_n_p_T)
fprintf('Cyp %f %f \n',C_y_p,C_y_p_T)
fprintf('Clbeta %f %f \n',C_l_beta,C_l_beta_T)
fprintf('Cnbeta %f %f \n',C_n_beta,C_n_beta_T)
fprintf('Cybeta %f %f \n',C_y_beta,C_y_beta_T)
fprintf('Cyr %f %f \n',C_y_r,C_y_r_T)
fprintf('Cnr %f %f \n',C_n_r,C_n_r_T)
fprintf('Clr %f %f \n',C_l_r,C_l_r_T)

%%%Plot Short Period Approximation
%plot(f1,real_lambda1_vec,imag_lambda1_vec,'k-','LineWidth',2)
%plot(f1,real_lambda2_vec,imag_lambda2_vec,'k-','LineWidth',2)
p3 = plot(f1,real_lambda1_Avec,imag_lambda1_Avec,'k--','LineWidth',2);
p3 = plot(f1,real_lambda2_Avec,imag_lambda2_Avec,'k--','LineWidth',2);

%%%Phugoid
%plot(f5,real_lambdalp1_vec,imag_lambdalp1_vec,'k-','LineWidth',2)
%plot(f5,real_lambdalp2_vec,imag_lambdalp2_vec,'k-','LineWidth',2)
plot(f5,real_lambdalp1_Avec,imag_lambdalp1_Avec,'k--','LineWidth',2)
plot(f5,real_lambdalp2_Avec,imag_lambdalp2_Avec,'k--','LineWidth', ...
     2)

 %%%Dutch Roll
plot(f7,real_lambdafl1_vec,imag_lambdafl1_vec,'k--','LineWidth',2)
plot(f7,real_lambdafl2_vec,imag_lambdafl2_vec,'k--','LineWidth',2)
%plot(f7,real_lambdafl1_Avec,imag_lambdafl1_Avec,'k--','LineWidth',2)
%plot(f7,real_lambdafl2_Avec,imag_lambdafl2_Avec,'k--','LineWidth',2)

%%%Roll
%plot(f6,Nc,roll_vec,'k-','LineWidth',2)
plot(f6,Nc,roll_Avec,'k--','LineWidth',2)

%%%Spiral
%plot(f8,Nc,spiral_vec,'k-','LineWidth',2)
plot(f8,Nc,spiral_Avec,'k--','LineWidth',2)

if W2W
  %%%Plot the numerical values
  ac1 = [-5.922307 + 9.223556i, -5.922307 + -9.223556i];
  ac2w2w = [-6.006579 + 9.216224i 	,-6.006579 + -9.216224i 	];
  ac3w2w = [-6.044713 + 9.213757i 	,-6.044713 + -9.213757i 	];
  ac4w2w = [-6.062133 + 9.209107i 	,-6.062133 + -9.209107i ];
  ac5w2w = [-6.075273 + 9.208345i ,-6.075273 + -9.208345i ];
  w2w = [ac1;ac2w2w;ac3w2w;ac4w2w;ac5w2w];
  w2wsp = round(w2w*1000)/1000;
  
  p1 = plot(f1,w2w,'kx','MarkerSize',10,'LineWidth',2);
  
  %%%%Slow longitudinal mode
  ac1 = [-0.032646 + 0.610550i ,	  -0.032646 + -0.610550i];
  ac2w2w = [-0.032284 + 0.608451i,  	-0.032284 + -0.608451i];
  ac3w2w = [	-0.032133 + 0.607567i, 	-0.032133 + -0.607567i 	];
  ac4w2w =  [	-0.032052 + 0.607074i ,  -0.032052 + -0.607074i];
  ac5w2w =  [	-0.031999 + 0.606755i  ,	-0.031999 + -0.606755i ];
  w2w = [ac1;ac2w2w;ac3w2w;ac4w2w;ac5w2w];
  w2wlp = round(w2w*1000)/1000;
  plot(f5,w2w,'kx','MarkerSize',10,'LineWidth',2);
  
  %%%%%Roll mode
  w2w = [-16.934008 + 0.000000i 	;-8.001232 + 0.000000i ; ...
	 -7.664131 + 0.000000i ;	-7.592755 + 0.000000i 	;-7.571826 + 0.000000i ];
 w2wroll = round(w2w*1000)/1000;
 plot(f6,w2w,'kx','MarkerSize',10,'LineWidth',2);
  %%%Dutch Roll Mode
  ac1 = [-0.418451 + 2.314802i ,	  -0.418451 - 2.314802i 	];
  ac2 = [-0.237731 + 0.873577i ,	  -0.237731 - 0.873577i 	];
  ac3 = [-0.227077 + 0.557977i , -0.227077 - 0.557977i];
  ac4= [-0.223070 + 0.410829i,   -0.223070 - 0.410829i];
  ac5 = [-0.220666 + 0.324168i ,   -0.220666 - 0.324168i ];
  w2w = [ac1;ac2;ac3;ac4;ac5];
  w2wfl = round(w2w*1000)/1000;
  plot(f7,w2w,'kx','MarkerSize',10,'LineWidth',2);
  
  xlim(f7,[-1.4 0.2])
  xl = xlim(f7);
  yl = ylim(f7);
  plot(f7,[xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot(f7,[0 0],[yl(1) yl(2)],'k--','LineWidth',1)
  grid(f7,'off')

  %%%Spiral mode
  w2w = [-0.028812 + 0.000000i;	0.029281 + 0.000000i ;	0.031599 + 0.000000i; 	0.030842 + 0.000000i; 	0.028222 + 0.000000i ];
  w2wspiral = round(w2w*1000)/1000;
  plot(f8,w2w,'kx','MarkerSize',10,'LineWidth',2);

  Names = {};
  Names = [Names;'Root (Approximate)';'Root (Numerical)';'Natural Frequency';'Damping'];
  sp = 'Short Period';
  ph = 'Phugoid';
  dr = 'Dutch Roll';
  roll = 'Roll';
  spir = 'Spiral';
  Column1 = {};
  Column2 = {};
  Column1 = [Column1;' ';sp;sp;sp;sp;ph;ph;ph;ph;dr;dr;dr;dr;roll;roll;spir;spir];
  Column2 = [Column2;'Number of Aircraft';Names;Names;Names;'Root (Approximate)';'Root (Numerical)';'Root (Approximate)';'Root (Numerical)'];
  
  Row1 = {'1','2','3','4','5'};
  Row2 = {};
  Row3 = {};
  Row4 = {};
  Row5 = {};
  %%%Short period  
  real_lambda1_Avec=round(real_lambda1_Avec*1000)/1000;
  real_lambda2_Avec=round(real_lambda2_Avec*1000)/1000;
  imag_lambda1_Avec=round(imag_lambda1_Avec*1000)/1000;
  imag_lambda2_Avec=round(imag_lambda2_Avec*1000)/1000;
  for nn = 1:5
    Row2 = [Row2,[num2str(real_lambda1_Avec(nn)),'+',num2str(imag_lambda1_Avec(nn)),'i']];
    Row3 = [Row3,[num2str(w2wsp(nn,1))]];
    s1 = w2wsp(nn,1);
    s2 = w2wsp(nn,2);
    A = (s1+s2)/2;
    B = (s1-s2)/2;
    zed = sqrt(A^2/(A^2-B^2));
    wn = sqrt(A^2/zed^2);
    zed = round(zed*1000)/1000;
    wn = round(wn*1000)/1000;
    Row4 = [Row4,num2str(wn)];
    Row5 = [Row5,num2str(zed)];
  end
  data = [Row1;Row2;Row3;Row4;Row5];
  Row2 = {};
  Row3 = {};
  Row4 = {};
  Row5 = {};
  %%%Phugoid period  
  real_lambdalp1_Avec=round(real_lambdalp1_Avec*1000)/1000;
  real_lambdalp2_Avec=round(real_lambdalp2_Avec*1000)/1000;
  imag_lambdalp1_Avec=round(imag_lambdalp1_Avec*1000)/1000;
  imag_lambdalp2_Avec=round(imag_lambdalp2_Avec*1000)/1000;
  for nn = 1:5
    Row2 = [Row2,[num2str(real_lambdalp1_Avec(nn)),'+',num2str(imag_lambdalp1_Avec(nn)),'i']];
    Row3 = [Row3,[num2str(w2wlp(nn,1))]];
    s1 = w2wlp(nn,1);
    s2 = w2wlp(nn,2);
    A = (s1+s2)/2;
    B = (s1-s2)/2;
    zed = sqrt(A^2/(A^2-B^2));
    wn = sqrt(A^2/zed^2);
    zed = round(zed*1000)/1000;
    wn = round(wn*1000)/1000;
    Row4 = [Row4,num2str(wn)];
    Row5 = [Row5,num2str(zed)];
  end
  data = [data;Row2;Row3;Row4;Row5];
  Row2 = {};
  Row3 = {};
  Row4 = {};
  Row5 = {};
  %%%Dutch Roll
  real_lambdafl1_vec=round(real_lambdafl1_vec*1000)/1000;
  real_lambdafl2_vec=round(real_lambdafl2_vec*1000)/1000;
  imag_lambdafl1_vec=round(imag_lambdafl1_vec*1000)/1000;
  imag_lambdafl2_vec=round(imag_lambdafl2_vec*1000)/1000;
  for nn = 1:5
    Row2 = [Row2,[num2str(real_lambdafl1_vec(nn)),'+',num2str(imag_lambdafl1_vec(nn)),'i']];
    Row3 = [Row3,[num2str(w2wfl(nn,1))]];
    s1 = w2wfl(nn,1);
    s2 = w2wfl(nn,2);
    A = (s1+s2)/2;
    B = (s1-s2)/2;
    zed = sqrt(A^2/(A^2-B^2));
    wn = sqrt(A^2/zed^2);
    zed = round(zed*1000)/1000;
    wn = round(wn*1000)/1000;
    Row4 = [Row4,num2str(wn)];
    Row5 = [Row5,num2str(zed)];
  end
  data = [data;Row2;Row3;Row4;Row5];
  Row2 = {};
  Row3 = {};
  %%%Roll
  roll_Avec = round(roll_Avec*1000)/1000;
  for nn = 1:5
    Row2 = [Row2,[num2str(roll_Avec(nn))]];
    Row3 = [Row3,[num2str(w2wroll(nn,1))]];
  end
  data = [data;Row2;Row3];
  Row2 = {};
  Row3 = {};
  %%%Spiral
  spiral_Avec = round(spiral_Avec*1000)/1000;
  for nn = 1:5
    Row2 = [Row2,[num2str(spiral_Avec(nn))]];
    Row3 = [Row3,[num2str(w2wspiral(nn,1))]];
  end
  data = [data;Row2;Row3];
  xlsmatrix = [Column1,Column2,data];
  
else  %%%Plot the numerical values
  ac1 = [-5.922307 + 9.223556i, -5.922307 + -9.223556i];
  ac2t2t = [-3.5289 + 2.184i,-3.5289 - 2.184i];
  ac3t2t = [-3.21967 + 0.8582i,-3.21967 - 0.8582i];
  ac4t2t = [-3.6919,-2.404];
  ac5t2t = [-3.8935,-1.9547];
  ac2t2t = [-7.013519 + 0.996043i 	,  -7.013519 + -0.996043i 	];
  ac3t2t = [-8.490514 + 0.000000i ,  -5.383219 + 0.000000i 	];
  ac4t2t = [-8.583895 + 0.000000i 	,	-5.085817 + 0.000000i ];
  ac5t2t = [-8.521743 + 0.000000i ,		-4.953838 + 0.000000i ];
  t2t = [ac1;ac2t2t;ac3t2t;ac4t2t;ac5t2t];
  t2tsp = round(t2t*1000)/1000;
  p2 = plot(f1,t2t,'ks','MarkerSize',10,'LineWidth',2);
  
  xlim(f1,[-9 1])
  xl = xlim(f1);
  yl = ylim(f1);
  plot(f1,[xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot(f1,[0 0],[yl(1) yl(2)],'k--','LineWidth',1)  
  grid(f1,'off')
  
  ac1 = [  -0.032646 + 0.610550i ,  -0.032646 + -0.610550i];
  ac2 = [-0.055057 + 0.091021i 	,  -0.055057 + -0.091021i];
  ac3 = [-0.240333 + 0.000000i 	,  0.115779 + 0.000000i 	];
  ac4 = [ -0.267625 + 0.000000i,  0.137605 + 0.000000i 	];
  ac5 = [-0.276049 + 0.000000i  ,  0.142405 + 0.000000i ];
  t2t = [ac1;ac2;ac3;ac4;ac5];
  t2tph = round(t2t*1000)/1000;
  p2 = plot(f5,t2t,'ks','MarkerSize',10,'LineWidth',2);
  
  %xlim(f5,[-0.3 0.2])
  xl = xlim(f5);
  yl = ylim(f5);
  plot(f5,[xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot(f5,[0 0],[yl(1) yl(2)],'k--','LineWidth',1)
  grid(f5,'off')
  
  %%%Roll modes
  t2t = [-16.934008 + 0.000000i ;	-14.453643 + 0.000000i ;	-13.156881 + 0.000000i ;	-12.225128 + 0.000000i ;	-11.488919 + 0.000000i ];
  t2troll = round(t2t*1000)/1000;
  plot(f6,t2t,'ks','MarkerSize',10,'LineWidth',2);
  
  %%%Dutch Roll Modes
  ac1 = [-0.418451 + 2.314802i,-0.418451 + -2.314802i];
  ac2 = [-0.265336 + 0.968422i , 	-0.265336 + -0.968422i];
  ac3 = [-0.307933 + 0.384970i , 	-0.307933 + -0.384970i];
  ac4 = [-0.855535 + 0.000000i, 	-0.230157 + 0.000000i];
  ac5 = [-1.134651 + 0.000000i , 	-0.202931 + 0.000000i ];
  t2t = [ac1;ac2;ac3;ac4;ac5];
  t2tfl = round(t2t*1000)/1000;
  plot(f7,t2t,'ks','MarkerSize',10,'LineWidth',2);
  
  %%%Spiral Modes
  t2t = [-0.028812 + 0.000000i	-0.102091 + 0.000000i 	-0.005344 + 0.000000i 	0.451871 + 0.000000i 	0.569203 + 0.000000i ];
  plot(f8,t2t,'ks','MarkerSize',10,'LineWidth',2);
  t2tspiral = round(t2t*1000)/1000;
  
  Row1 = {'1','2','3','4','5'};
  Row2 = {};
  Row3 = {};
  Row4 = {};
  Row5 = {};
  %%%Short period  
  real_lambda1_Avec=round(real_lambda1_Avec*1000)/1000;
  real_lambda2_Avec=round(real_lambda2_Avec*1000)/1000;
  imag_lambda1_Avec=round(imag_lambda1_Avec*1000)/1000;
  imag_lambda2_Avec=round(imag_lambda2_Avec*1000)/1000;
  for nn = 1:5
      if imag_lambda1_Avec(nn) == 0
        Row2 = [Row2,[num2str(real_lambda1_Avec(nn)),'  ',num2str(real_lambda2_Avec(nn))]];
        Row3 = [Row3,[num2str(t2tsp(nn,1)),'  ',num2str(t2tsp(nn,2))]];
      else
        Row2 = [Row2,[num2str(real_lambda1_Avec(nn)),'+',num2str(imag_lambda1_Avec(nn)),'i']];
        Row3 = [Row3,[num2str(t2tsp(nn,1))]];
      end
    s1 = t2tsp(nn,1);
    s2 = t2tsp(nn,2);
    A = (s1+s2)/2;
    B = (s1-s2)/2;
    zed = sqrt(A^2/(A^2-B^2));
    wn = sqrt(A^2/zed^2);
    zed = round(zed*1000)/1000;
    wn = round(wn*1000)/1000;
    Row4 = [Row4,num2str(wn)];
    Row5 = [Row5,num2str(zed)];
  end
  data = [Row1;Row2;Row3;Row4;Row5];
  Row2 = {};
  Row3 = {};
  Row4 = {};
  Row5 = {};
  %%%Phugoid period     
  real_lambdalp1_Avec=round(real_lambdalp1_Avec*1000)/1000;
  real_lambdalp2_Avec=round(real_lambdalp2_Avec*1000)/1000;
  imag_lambdalp1_Avec=round(imag_lambdalp1_Avec*1000)/1000;
  imag_lambdalp2_Avec=round(imag_lambdalp2_Avec*1000)/1000;
  for nn = 1:5
      s1 = t2tph(nn,1);
      s2 = t2tph(nn,2);
      A = (s1+s2)/2;
      B = (s1-s2)/2;
      if imag_lambdalp1_Avec(nn) == 0
        Row2 = [Row2,[num2str(real_lambdalp2_Avec(nn)),'  ',num2str(real_lambdalp1_Avec(nn))]];
        Row3 = [Row3,[num2str(t2tph(nn,1)),'  ',num2str(t2tph(nn,2))]];
        zed = sqrt(B^2/(B^2-A^2));
        wn = sqrt(B^2/zed^2);
      else
        Row2 = [Row2,[num2str(real_lambdalp1_Avec(nn)),'+',num2str(imag_lambdalp1_Avec(nn)),'i']];
        Row3 = [Row3,[num2str(t2tph(nn,1))]];
        zed = sqrt(A^2/(A^2-B^2));
        wn = sqrt(A^2/zed^2);
      end
      zed = round(zed*1000)/1000;
      wn = round(wn*1000)/1000;
    Row4 = [Row4,num2str(wn)];
    Row5 = [Row5,num2str(zed)];
  end
  data = [data;Row2;Row3;Row4;Row5];
  Row2 = {};
  Row3 = {};
  Row4 = {};
  Row5 = {};
  %%%Dutch Roll
  real_lambdafl1_vec=round(real_lambdafl1_vec*1000)/1000;
  real_lambdafl2_vec=round(real_lambdafl2_vec*1000)/1000;
  imag_lambdafl1_vec=round(imag_lambdafl1_vec*1000)/1000;
  imag_lambdafl2_vec=round(imag_lambdafl2_vec*1000)/1000;
  for nn = 1:5
    s1 = t2tfl(nn,1);
    s2 = t2tfl(nn,2);
    A = (s1+s2)/2;
    B = (s1-s2)/2;
    if imag_lambdafl1_vec(nn) == 0
        Row2 = [Row2,[num2str(real_lambdafl1_vec(nn)),'  ',num2str(real_lambdafl2_vec(nn))]];
        Row3 = [Row3,[num2str(t2tfl(nn,1)),'  ',num2str(t2tfl(nn,2))]]; 
        zed = sqrt(A^2/(A^2-B^2));
        wn = sqrt(A^2/zed^2);
    else
        Row2 = [Row2,[num2str(real_lambdafl1_vec(nn)),'+',num2str(imag_lambdafl1_vec(nn)),'i']];
        Row3 = [Row3,[num2str(t2tfl(nn,1))]]; 
        zed = sqrt(A^2/(A^2-B^2));
        wn = sqrt(A^2/zed^2);
    end
    zed = round(zed*1000)/1000;
    wn = round(wn*1000)/1000;
    Row4 = [Row4,num2str(wn)];
    Row5 = [Row5,num2str(zed)];
  end
  data = [data;Row2;Row3;Row4;Row5];
  Row2 = {};
  Row3 = {};
  %%%Roll
  roll_Avec = round(roll_Avec*1000)/1000;
  for nn = 1:5
    Row2 = [Row2,[num2str(roll_Avec(nn))]];
    Row3 = [Row3,[num2str(t2troll(nn,1))]];
  end
  data = [data;Row2;Row3];
  Row2 = {};
  Row3 = {};
  %%%Spiral
  spiral_Avec = round(spiral_Avec*1000)/1000;
  for nn = 1:5
    Row2 = [Row2,[num2str(spiral_Avec(nn))]];
    Row3 = [Row3,[num2str(t2tspiral(nn))]];
  end
  data = [data;Row2;Row3];
  
  xlsmatrix = [xlsmatrix,data]
  
end

end

legend(f1,[p1(1),p2(1),p3],'Wing Tip to Wing Tip','Tip to Tail','Approximate Solution')
legend(f5,[p1(1),p2(1),p3],'Wing Tip to Wing Tip','Tip to Tail','Approximate Solution')
legend(f6,[p1(1),p2(1),p3],'Wing Tip to Wing Tip','Tip to Tail','Approximate Solution')
legend(f7,[p1(1),p2(1),p3],'Wing Tip to Wing Tip','Tip to Tail','Approximate Solution')
legend(f8,[p1(1),p2(1),p3],'Wing Tip to Wing Tip','Tip to Tail','Approximate Solution')
legend(f2,'Wing Tip to Wing Tip','Tip to Tail')
%close all

  
