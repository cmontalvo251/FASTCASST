purge

b = 2.04;
Iyy = 0.5111;
rho = 1.220281;
V = sqrt(19.98^2+1.068^2);
S = 0.6558;
c = 0.3215;
C_l_p = -0.4625;
C_L_alpha = 5.195;
mass = 55.07/9.81;

A = b*c/(4*Iyy)*rho*V*S*C_l_p

%%2 a/c

I2 = 2*(Iyy + mass*(b/2)^2);
A = b/(2*I2)*rho*V*S*(c*C_l_p-b/2*C_L_alpha)

%%N a/c
Nvec = 1:10;
rollmodes = zeros(1,length(Nvec));
for ndx = 1:length(Nvec)
  N = Nvec(ndx);
  r = linspace(-N*b/2+b/2,N*b/2-b/2,N);
  IN = N*Iyy + mass*sum(r.^2);
  rollmodes(ndx) = rho*V*S/(2*IN)*(N*b*c/2*C_l_p - C_L_alpha*sum(r.^2));
end

plot(Nvec,rollmodes)