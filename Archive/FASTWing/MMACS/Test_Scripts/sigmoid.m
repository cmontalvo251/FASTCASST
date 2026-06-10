breakpt = 0.3;
lowerbound = 0.0001;
upperbound = 0.99;
b = log(1/lowerbound-1);
a = (log(upperbound)-b)/breakpt;

delxz = linspace(0,1,100);

WAYPOINT =  1-1./(1+exp(a.*abs(delxz)+b));

plottool(1,'Sigmoid',18,'\Delta d (m)','W_{connect}');
plot(delxz,WAYPOINT,'k-','LineWidth',2)