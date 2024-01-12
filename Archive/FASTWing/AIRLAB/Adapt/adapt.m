%%Testing Adaptive Element
purge

global u A B D

%%Define System
%Spring Mass Damper System with a Forcing function and a disturbance
k = 100;
c = 10;
A = [0 1;-k -c];
B = [0;1];
u = 100;
D = [0;100];

%%Setup Integration Parameters
timestep = 0.001;
t0 = 0;
time = t0:timestep:2;
xout = zeros(2,length(time));
xExact = xout;
xDiscrete = xout;
xEstimate = xout;
Destimate = xout;
wk = xout;
x0 = [0;5];
x = x0;
xDiscrete(:,1) = x0;
xEstimate(:,1) = x0;
functionHandle = @Spring;
Nadapt = [0.001 0;0 0.001];

%%Compute Discrete matrices
Ad = expm(A*timestep);
Dd = inv(A)*(expm(A*timestep)-eye(2));
Bd = Dd*B;

for ii = 1:length(time)
  %%RK4 Integration
  xout(:,ii) = x;
  xdot1 = feval(functionHandle, time(ii), x);
  xdot2 = feval(functionHandle, time(ii) + (.5*timestep), x + (xdot1*.5*timestep));
  xdot3 = feval(functionHandle, time(ii) + (.5*timestep), x + (xdot2*.5*timestep));
  xdot4 = feval(functionHandle, time(ii) + timestep, x + (xdot3*timestep));
  xdotRK4 = (1/6) * (xdot1 + (2*xdot2) + (2*xdot3) + xdot4);
  nextstate = x + (timestep * xdotRK4);
  x = nextstate;
  %%%Exact Solution
  xExact(:,ii) = expm(A*(time(ii)-t0))*x0 + inv(A)*(expm(A*(time(ii)-t0))-eye(2))*(B*u+D);
  %%Discrete Solution
  if ii ~= length(time)
    xDiscrete(:,ii+1) = Ad*xDiscrete(:,ii) + Bd*u + Dd*D;
  end
  %%Estimating Disturbance
  if ii ~= length(time)
    xEstimate(:,ii+1) = Ad*xEstimate(:,ii) + Bd*u + wk(:,ii);
    wk(:,ii+1) = wk(:,ii) + Nadapt*(x-xEstimate(:,ii));
    %%Now Let's see if we can relate wk to D
    Destimate(:,ii+1) = inv(expm(A*timestep)-eye(2))*A*wk(:,ii+1);
  end
end
plottool(1,'Spring',12,'Time(s)','x and xdot');
p1 = plot(time,xout,'b-','LineWidth',2);
p2 = plot(time,xExact,'r-','LineWidth',2);
p3 = plot(time,xDiscrete,'g-','LineWidth',2);
p4 = plot(time,xEstimate,'m-','LineWidth',2);
legend([p1(1),p2(1),p3(1),p4(1)],'RK4','Exact','Discrete','Adaptive')

plottool(1,'Adapt',12,'Time(s)','Destimate')
plot(time,Destimate,'LineWidth',2)
