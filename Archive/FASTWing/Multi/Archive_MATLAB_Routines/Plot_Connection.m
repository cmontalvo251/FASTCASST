purge

KR = 0;
CR = 3;
FITMODES = 1;

Ctheta = 10.33623;
Ktheta = 2584.05625;
Cpsi = 10.33623;
Kpsi = 2584.05625;  
Cphi = 1.4884;
Kphi = 372.1041;
%%%Nominal Modes
flappingNom = -12+38.8i;
twistpitchNom = -22.5 + 98.5i;
leadlag = -12.5+77.2i;

if KR == 1
  load KR1.mat
end
if KR == 2
  load KR2.mat
end
if KR == 3
  load KR3.mat
end
if CR == 1
  load CR1.mat
end
if CR == 2
  load CR2.mat
end
if CR == 3
  load CR3.mat
end

% plottool(1,'Pitch',12,'Real','Imaginary');
% plot(pitch,'k-x','LineWidth',2)

% plottool(1,'Flapping',12,'Real','Imaginary');
% plot(flapping,'k-x','LineWidth',2)

% plottool(1,'Lead Lag',12,'Real','Imaginary');
% plot(leadlag,'k-x','LineWidth',2)

plottool(1,'All',12,'Real','Imaginary');

nums = 1000; %970
num = 1000;

p1 = plot(real(pitch(nums:end,1)),imag(pitch(nums:end,1)),'k-s','LineWidth',2);
plot(real(pitch(nums:end,2)),imag(pitch(nums:end,2)),'k-s','LineWidth',2);

p2 = plot(real(flapping(:,1)),imag(flapping(:,1)),'k-^','LineWidth',2);
plot(real(flapping(:,2)),imag(flapping(:,2)),'k-^','LineWidth',2);

p3 = plot(real(leadlag(1:num,1)),imag(leadlag(1:num,1)),'k-d','LineWidth',2);
plot(real(leadlag(1:num,2)),imag(leadlag(1:num,2)),'k-d','LineWidth',2);


if KR == 2
  %%%Try to fit this to a transfer Function
  %%Assume that T{s} = U/(Js^2+(C+CA)s+K)
  %%Then the modes are equal to 
  %%modes = -(C+CA+sqrt((C+CA)^2-4JK))/(2J)
  Ktheta = param;
  X = -Ktheta';
  H = [pitch(:,1).^2 pitch(:,1)];
  theta = inv(H'*H)*H'*X;
  %%%Make answers real
  theta = real(theta);
  J = theta(1)
  CT = theta(2);
  CA = CT-Ctheta
  modes1 = -((Ctheta+CA)+sqrt((Ctheta+CA)^2-4.*J.*Ktheta))./(2*J);
  modes2 = -((Ctheta+CA)-sqrt((Ctheta+CA)^2-4.*J.*Ktheta))./(2*J);
  %%%Plot this
  if FITMODES
  plot(real(modes1),imag(modes1),'r-','LineWidth',2);
  plot(real(modes2),imag(modes2),'r-','LineWidth',2);
  end
  xlim([-45 0])
  xl = xlim;
  yl = ylim;
  plot([xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot([0 0],[yl(1) yl(2)],'k--','LineWidth',1)
  grid off
end
if CR == 2
  %%%Try to fit this to a transfer Function
  %%Assume that T{s} = U/(Js^2+(C+CA)s+K)
  %%Then the modes are equal to 
  %%modes = -(C+CA+sqrt((C+CA)^2-4JK))/(2J)
  Ctheta = param;
  X = -Ktheta'-Ctheta'.*pitch(:,1);
  H = [pitch(:,1).^2 pitch(:,1)];
  theta = inv(H'*H)*H'*X;
  %%%Make answers real
  theta = real(theta);
  J = theta(1)
  CA = theta(2)
  modes1 = -((Ctheta+CA)+sqrt((Ctheta+CA).^2-4.*J.*Ktheta))./(2*J);
  modes2 = -((Ctheta+CA)-sqrt((Ctheta+CA).^2-4.*J.*Ktheta))./(2*J);
  %%%Plot this
  if FITMODES
  plot(real(modes1),imag(modes1),'r-','LineWidth',2);
  plot(real(modes2),imag(modes2),'r-','LineWidth',2);
  end
  J = 0.2570
  CA = 0.544
  modes1 = -((Ctheta+CA)+sqrt((Ctheta+CA).^2-4.*J.*Ktheta))./(2*J);
  modes2 = -((Ctheta+CA)-sqrt((Ctheta+CA).^2-4.*J.*Ktheta))./(2*J);
  %%%Plot this
  if FITMODES
  plot(real(modes1),imag(modes1),'g-','LineWidth',2);
  plot(real(modes2),imag(modes2),'g-','LineWidth',2);
  end
  xlim([-200 0])
  xl = xlim;
  yl = ylim;
  plot([xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot([0 0],[yl(1) yl(2)],'k--','LineWidth',1)
  grid off
end

if KR == 1
  %%%Try to fit this to a transfer Function
  %%Assume that T{s} = U/(Js^2+(C+CA)s+K)
  %%Then the modes are equal to 
  %%modes = -(C+CA+sqrt((C+CA)^2-4JK))/(2J)
  Kphi = param;
  X = -Kphi';
  H = [flapping(:,1).^2 flapping(:,1)];
  theta = inv(H'*H)*H'*X;
  %%%Make answers real
  theta = real(theta);
  J = theta(1)
  CT = theta(2);
  CA = CT-Cphi
  modes1 = -((Cphi+CA)+sqrt((Cphi+CA)^2-4.*J.*Kphi))./(2*J);
  modes2 = -((Cphi+CA)-sqrt((Cphi+CA)^2-4.*J.*Kphi))./(2*J);
  %%%Plot this
  if FITMODES
  plot(real(modes1),imag(modes1),'r-','LineWidth',2);
  plot(real(modes2),imag(modes2),'r-','LineWidth',2);
  end
  xlim([-25 5])
  xl = xlim;
  yl = ylim;
  plot([xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot([0 0],[yl(1) yl(2)],'k--','LineWidth',1)
  grid off
end
if CR == 1
  %%%Try to fit this to a transfer Function
  %%Assume that T{s} = U/(Js^2+(C+CA)s+K)
  %%Then the modes are equal to 
  %%modes = -(C+CA+sqrt((C+CA)^2-4JK))/(2J)
  Cphi = param;
  X = -Kphi'-Cphi'.*flapping(:,1);
  H = [flapping(:,1).^2 flapping(:,1)];
  theta = inv(H'*H)*H'*X;
  %%%Make answers real
  theta = real(theta);
  J = theta(1)
  CA = theta(2)
  modes1 = -((Cphi+CA)+sqrt((Cphi+CA).^2-4.*J.*Kphi))./(2*J);
  modes2 = -((Cphi+CA)-sqrt((Cphi+CA).^2-4.*J.*Kphi))./(2*J);
  %%%Plot this
  if FITMODES
  plot(real(modes1),imag(modes1),'r-','LineWidth',2);
  plot(real(modes2),imag(modes2),'r-','LineWidth',2);
  end
  xlim([-140 0])
  xl = xlim;
  yl = ylim;
  plot([xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot([0 0],[yl(1) yl(2)],'k--','LineWidth',1)
  grid off
end

if KR == 3
  %%%Try to fit this to a transfer Function
  %%Assume that T{s} = U/(Js^2+(C+CA)s+K)
  %%Then the modes are equal to 
  %%modes = -(C+CA+sqrt((C+CA)^2-4JK))/(2J)
  Kpsi = param;
  X = -Kpsi';
  H = [leadlag(:,1).^2 leadlag(:,1)];
  theta = inv(H'*H)*H'*X;
  %%%Make answers real
  theta = real(theta);
  J = theta(1)
  CT = theta(2);
  CA = CT-Cpsi
  modes1 = -((Cpsi+CA)+sqrt((Cpsi+CA)^2-4.*J.*Kpsi))./(2*J);
  modes2 = -((Cpsi+CA)-sqrt((Cpsi+CA)^2-4.*J.*Kpsi))./(2*J);
  %%%Plot this
  if FITMODES
  plot(real(modes1),imag(modes1),'r-','LineWidth',2);
  plot(real(modes2),imag(modes2),'r-','LineWidth',2);
 end
  xlim([-25 5])
  xl = xlim;
  yl = ylim;
  plot([xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot([0 0],[yl(1) yl(2)],'k--','LineWidth',1)
  grid off

end
if CR == 3
  %%%Try to fit this to a transfer Function
  %%Assume that T{s} = U/(Js^2+(C+CA)s+K)
  %%Then the modes are equal to 
  %%modes = -(C+CA+sqrt((C+CA)^2-4JK))/(2J)
  Cpsi = param;
  X = -Kpsi'-Cpsi'.*leadlag(:,1);
  H = [leadlag(:,1).^2 leadlag(:,1)];
  theta = inv(H'*H)*H'*X;
  %%%Make answers real
  theta = real(theta);
  J = theta(1)
  CA = theta(2)
  modes1 = -((Cpsi+CA)+sqrt((Cpsi+CA).^2-4.*J.*Kpsi))./(2*J);
  modes2 = -((Cpsi+CA)-sqrt((Cpsi+CA).^2-4.*J.*Kpsi))./(2*J);
  %%%Plot this
  if FITMODES
  plot(real(modes1),imag(modes1),'r-','LineWidth',2);
  plot(real(modes2),imag(modes2),'r-','LineWidth',2);
  end
  J = 0.4228
  CA = 0.121
  modes1 = -((Cpsi+CA)+sqrt((Cpsi+CA).^2-4.*J.*Kpsi))./(2*J);
  modes2 = -((Cpsi+CA)-sqrt((Cpsi+CA).^2-4.*J.*Kpsi))./(2*J);
  %%%Plot this
  if FITMODES
  plot(real(modes1),imag(modes1),'g-','LineWidth',2);
  plot(real(modes2),imag(modes2),'g-','LineWidth',2);
  end
  xlim([-250 0])
  xl = xlim;
  yl = ylim;
  plot([xl(1) xl(2)],[0 0],'k--','LineWidth',1)
  plot([0 0],[yl(1) yl(2)],'k--','LineWidth',1)
  grid off
end

legend([p1(1) p2(1) p3(1)],'Twist(pitch) Mode','Flapping Mode','Lead Lag Mode')

