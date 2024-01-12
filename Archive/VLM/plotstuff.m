purge
purge
load Lift.mat

plottool(1,'Lift',12,'Distance(m)','CL/CL0');
x = 5:-0.01:2.04;
H1 = Claequivvecm(1,:);
H1_0 = H1(1);
H100 = Claequivvecm(2,:); 
H100_0 = H100(1);
plot(x,H1./H1_0,'LineWidth',2)
plot(x,H100./H100_0,'g-','LineWidth',2)
legend('H = 1','H = 100')


