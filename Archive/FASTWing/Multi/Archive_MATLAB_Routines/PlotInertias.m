purge

T2T = 0;
W2W = 1;

N = 1:6;

Ixxvec = zeros(1,length(N));

for ii = 1:length(N)

  num_ac = N(ii);

  %%Compute composite c.g and distance vectors from c.g to each aircraft
  rcg2ac = zeros(2,num_ac);
  xyzAC = zeros(2,num_ac);
  idx = 0;
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
  Ix1 = 0.4497;
  Iy1 = 0.5111;
  Iz1 = 0.8470;
  mass = num_ac*mass1;

  I_b1 = [Ix1 0 0;0 Iy1 0;0 0 Iz1];
  %%%Create Parrallel Axis theorem for all of them
  I_b = zeros(3,3);
  for idx = 1:num_ac
    s = [0 0 rcg2ac(2,idx);0 0 -rcg2ac(1,idx);-rcg2ac(2,idx) rcg2ac(1,idx) 0];
    I_cg = I_b1 + s*s'*mass1;
    I_b = I_b + I_cg;
  end

  Ixxvec(ii) = I_b(1,1);
end
plottool(1,'Inertia Fit',12,'Number of Aircraft','Ixx (kg-m^2)');
plot(N-1,Ixxvec,'LineWidth',2)

alfa = (Ixxvec-Ix1*N)./(N.^2)
%Ixxfit = Ix1*N + alfa.*N.^2;
%alfbar = (alfa.*N.^2)/mass1
%alfbar = N.*(N.^2-1)/12
%alfa = alfbar*mass1./N.^2

%betabar = 1/12;
%alfbar = betabar*(N.^3-N)*(2.04)^2

%alfbar = alfa.*N.^2/mass1
wspan = 2.04;

Ixxfit = (Ix1 + mass1/12*wspan^2*(N.^2-1)).*N;

plot(N-1,Ixxfit,'r--','LineWidth',2)

