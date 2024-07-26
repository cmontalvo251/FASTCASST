purge

%%%SHORT PERIOD%%%

ac1 = [-5.8096 + 8.9629i,-5.8096 - 8.9629i];
ac2w2w = [-5.903 + 8.988i,-5.903 - 8.988i];
ac3w2w = [-5.937 + 8.999i,-5.938 - 8.999i];
ac4w2w = [-5.9584 + 9.0056i,-5.9584 - 9.0056i];
ac5w2w = [-5.9719 + 9.0091i,-5.9719 - 9.0091i];

w2w = [ac1;ac2w2w;ac3w2w;ac4w2w;ac5w2w];

ac2t2t = [-3.5289 + 2.184i,-3.5289 - 2.184i];
ac3t2t = [-3.21967 + 0.8582i,-3.21967 - 0.8582i];
ac4t2t = [-3.6919,-2.404];
ac5t2t = [-3.8935,-1.9547];

t2t = [ac1;ac2t2t;ac3t2t;ac4t2t;ac5t2t];

plottool(1,'Short Period',12,'Real Axis','Imaginary Axis');
p1 = plot(w2w,'k-x','MarkerSize',10,'LineWidth',2);
p2 = plot(t2t,'k--x','MarkerSize',10,'LineWidth',2);
p3 = plot(ac1,'ks','MarkerSize',10,'LineWidth',2);
legend([p1(1),p2(1),p3(1)],'Wingtip to Wingtip','Tip to Tail','Single Aircraft')

%%%Roll Mode
w2w = [-3.2001,-3.3514,-3.381,-3.3455,-3.3266];
t2t = [-3.2001,-2.78,-2.676,-2.626,-2.60122];
num_ac = 1:5;

plottool(1,'Short Period',12,'Number of Aircraft','Roll Mode');
plot(num_ac,w2w,'kx-','LineWidth',2,'MarkerSize',10)
plot(num_ac,t2t,'kx--','LineWidth',2,'MarkerSize',10)
legend('Wingtip to Wingtip','Tip To Tail')

%%%No interact Spiral
w2w = [-0.0424,0.0372408082,0.0379474336,0.0341526827,0.0297635224];
t2t = [-0.0424,-0.1535,-0.2233,-0.30122,-0.3972];

num_ac = 1:5;

plottool(1,'Spiral',12,'Number of Aircraft','Spiral Mode');
plot(num_ac,w2w,'kx-','LineWidth',2,'MarkerSize',10)
plot(num_ac,t2t,'kx--','LineWidth',2,'MarkerSize',10)
legend('Wingtip to Wingtip','Tip To Tail')

%%Explanation of Roll Mode

%????

%%Dutch Roll Mode
ac1 = [-0.1506 + 2.1048i,-0.1506 - 2.1048i];
ac2w2w = [-0.2274 + 0.50676i,-0.2274 - 0.50676i];
ac3w2w = [-0.2138 + 0.3244i,-0.2138 - 0.3244i];
ac4w2w = [-0.2035 + 0.2449i,-0.2035 - 0.2449i];
ac5w2w = [-0.1978 + 0.1988i,-0.1978 - 0.1988i];

w2w = [ac1;ac2w2w;ac3w2w;ac4w2w;ac5w2w];

ac2t2t = [-0.0393 + 0.308i,-0.0393 - 0.308i];
ac3t2t = [-0.02977 + 0.475i,-0.02977 - 0.475i];
ac4t2t = [-0.03203 + 0.5532i,-0.03203 - 0.5532i];
ac5t2t = [-0.0412 + 0.6001i,-0.0412 - 0.6001i];

t2t = [ac1;ac2t2t;ac3t2t;ac4t2t;ac5t2t];

plottool(1,'Dutch Roll',12,'Real Axis','Imaginary Axis');
p1 = plot(w2w,'k-x','MarkerSize',10,'LineWidth',2);
p2 = plot(t2t,'k--x','MarkerSize',10,'LineWidth',2);
p3 = plot(ac1,'ks','MarkerSize',10,'LineWidth',2);
legend([p1(1),p2(1),p3(1)],'Wingtip to Wingtip','Tip to Tail','Single Aircraft')

%%Phugoid
ac1 = [-0.0601 + 0.6223i,-0.0601 - 0.6223i];
ac2w2w = [-0.0572 + 0.6192i,-0.0572 - 0.6192i];
ac3w2w = [-0.0559 + 0.6178i,-0.0559 - 0.6178i];
ac4w2w = [-0.0552 + 0.617i,-0.0552 - 0.617i];
ac5w2w = [-0.0551 + 0.616i,-0.0551 - 0.616i];

w2w = [ac1;ac2w2w;ac3w2w;ac4w2w;ac5w2w];

ac2t2t = [-0.1911 + 0.3596i,-0.1911 - 0.3596i];
ac3t2t = [-0.2777 + 0.0806i,-0.2777 - 0.0806i];
ac4t2t = [-0.5971,-0.0654];
ac5t2t = [-0.786,0.0095];

t2t = [ac1;ac2t2t;ac3t2t;ac4t2t;ac5t2t];

plottool(1,'Phugoid',12,'Real Axis','Imaginary Axis');
p1 = plot(w2w,'k-x','MarkerSize',10,'LineWidth',2);
p2 = plot(t2t,'k--x','MarkerSize',10,'LineWidth',2);
p3 = plot(ac1,'ks','MarkerSize',10,'LineWidth',2);
legend([p1(1),p2(1),p3(1)],'Wingtip to Wingtip','Tip to Tail','Single Aircraft')

%%Spiral
ac1 = [-0.0424,-0.0424];
ac2w2w = [0.0756 + 0.1748i,0.0756 - 0.1748i];
ac3w2w = [0.0659 + 0.1138i,0.0659 - 0.1138i];
ac4w2w = [0.0535 + 0.0799i,0.0535 - 0.0799i];
ac5w2w = [0.0438 + 0.0594i,0.0438 - 0.0594i];

w2w = [ac1;ac2w2w;ac3w2w;ac4w2w;ac5w2w];

ac2t2t = [0.09484 + 1.9754i,0.09484 - 1.9754i];
ac3t2t = [0.278 + 2.367i,0.278 - 2.367i];
ac4t2t = [0.5293 + 2.625i,0.5293 - 2.625i];
ac5t2t = [0.7716 + 2.702i,0.7716 - 2.702i];

% -0.0424
% -0.1535
% -0.2233
% -0.30122
% -0.3972

t2t = [ac1;ac2t2t;ac3t2t;ac4t2t;ac5t2t]

plottool(1,'Spiral',12,'Real Axis','Imaginary Axis');
p1 = plot(w2w,'k-x','MarkerSize',12,'LineWidth',2);
p2 = plot(t2t,'k--x','MarkerSize',12,'LineWidth',2);
p3 = plot(ac1,[0,0],'ks','MarkerSize',12,'LineWidth',2);
%%%No interact Spiral
w2w = [-0.0424,0.0372408082,0.0379474336,0.0341526827,0.0297635224];
w2w = [w2w'+0.01i,w2w'+0.01i];
t2t = [-0.0424,-0.1535,-0.2233,-0.30122,-0.3972];
t2t = [t2t'+0.01i,t2t'+0.01i]
p4 = plot(w2w,'k-+','MarkerSize',12,'LineWidth',2);
p5 = plot(t2t,'k--+','MarkerSize',12,'LineWidth',2);
legend([p1(1),p2(1),p3(1)],'Wingtip to Wingtip','Tip to Tail','Single Aircraft')
