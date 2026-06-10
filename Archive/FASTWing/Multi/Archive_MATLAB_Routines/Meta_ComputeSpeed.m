purge

global NSTATES num_ac mass I_b I_b_inv rcg2ac phic thetac

N = 1:5;

wnq_vec = zeros(1,length(N)); 
kpz_vec = zeros(1,length(N)); 
wnp_vec = zeros(1,length(N)); 
kppsi_vec = zeros(1,length(N)); 
qmax_vec = zeros(1,length(N)); 
pmax_vec = zeros(1,length(N)); 

W2W = 1;

%system('make rebuild');
config_data = {'20 20 !Grid_Size_of_Configuration'};
for ii = 1:20
  config_data = [config_data;num2str(zeros(1,10))];
end
config_data = [config_data;'10000 10000 !KLtangent_KLnormal';'40 40 !CLtangent_CLnormal';'1000 !Maximum_Magnet_Force';'372.1041 2584.05625 2584.05625 !KR';'1.4884 10.33623 10.33623 !CR'];

for num_ac = N
  
  disp(['N = ',num2str(num_ac)])

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

  disp('Compute Pitch Rate Dynamics')
  phic = 0;
  thetac = 30*pi/180;
  [wnq,kpz,d,d,qmax,d] = ComputeSpeed();
  %%Then compute maximum roll rate
  disp('Compute Roll Rate Dynamics')
  thetac = 0;
  phic = 30*pi/180;
  [d,d,wnp,kppsi,d,pmax] = ComputeSpeed();
  
  idx = num_ac;
  wnq_vec(idx) = wnq;
  kpz_vec(idx) = kpz;
  wnp_vec(idx) = wnp;
  kppsi_vec(idx) = kppsi;
  qmax_vec(idx) = qmax;
  pmax_vec(idx) = pmax;
  
end


plottool(1,'Pmax',12,'N','Pmax')
plot(pmax_vec,'b-','LineWidth',2)

l = 2:3;

pbar = mean(-1./(N(l)-1).*log(pmax_vec(l)/pmax_vec(1)));

ptilde = pmax_vec(1)*exp(-pbar*(N-1));

plot(ptilde,'r-','LineWidth',2)

ptilde2 = pmax_vec(1)./N.^2;

plot(ptilde2,'g-','LineWidth',2)

ptilde3 = pmax_vec(1)./N.^3;

plot(ptilde3,'g--','LineWidth',2)

pbarN = mean(log(pmax_vec(1)./pmax_vec(2:end))./log(N(2:end)));

ptildeN = pmax_vec(1)./N.^pbarN;

plot(ptildeN,'k-','LineWidth',2)
