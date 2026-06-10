%%Ok let's examine splitting the aero up
purge

x = 0;y=0;z=0;phi=0;theta=0;psi=0;

u = 8;
v=0; %1
w=0; 
p=1;
q=0;
r=0;

state = [x,y,z,phi,theta,psi,u,v,w,p,q,r];

da = 0;
dt = 0;
dr = 0;
de = 0;

controls = [da;dt;dr;de];

XYZa = zeros(6,21);
LMNa = zeros(6,21);
vec = zeros(1,21);

for ii = 1:21
  q = ii;
  vec(ii) = q;
  state = [x,y,z,phi,theta,psi,u,v,w,p,q,r];
  controls = [da;dt;dr;de];
  %First Split Aero
  [XYZa(1:3,ii),LMNa(1:3,ii)] = Split(state,controls);
  %Now Combined Aero
  [XYZa(4:6,ii),LMNa(4:6,ii)] = Combined(state,controls);
end

%%What to plot
plottool(1,'Xforce',12);
plot(vec,XYZa(1,:),'b-')
plot(vec,XYZa(4,:),'r-')
legend('Split','Combined')
plottool(1,'Yforce',12);
plot(vec,XYZa(2,:),'b-')
plot(vec,XYZa(5,:),'r-')
legend('Split','Combined')
plottool(1,'Zforce',12);
plot(vec,XYZa(3,:),'b-')
plot(vec,XYZa(6,:),'r-')
legend('Split','Combined')
plottool(1,'Xmoment',12);
plot(vec,LMNa(1,:),'b-')
plot(vec,LMNa(4,:),'r-')
legend('Split','Combined')
plottool(1,'Ymoment',12);
plot(vec,LMNa(2,:),'b-')
plot(vec,LMNa(5,:),'r-')
legend('Split','Combined')
plottool(1,'Zmoment',12);
plot(vec,LMNa(3,:),'b-')
plot(vec,LMNa(6,:),'r-')
legend('Split','Combined')