
t = 0:timestep:10;

x0 = 5;
xdot0 = 50;

A = [0 1;-100 -2];

x = zeros(2,length(t));

for ii = 1:length(t)
    x(:,ii) = expm(A.*t(ii))*[x0;xdot0];
end


plot(t,x(1,:))