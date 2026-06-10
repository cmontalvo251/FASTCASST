purge

A = dlmread('Fmeta.txt');

FMETA00 = A(:,1);
FMETA10 = A(:,2);

L0 = FMETA00;
M0 = FMETA10;

Body3 = L0;
Body3Body2 = M0;

plot(Body3,'b-')
hold on
plot(Body3Body2,'r-')
legend('Body3','Body3Body2')

break

id = A(:,2);
F = A(:,1);
t = A(:,3);

plottool(1,'id',12,'T (sec)','Independent Variable');
plot(t,id,'LineWidth',2)

%%%Test with seed data
%F = 2*id;

%%%We want F = id*K or X = H*theta
%%%Truncate fit after 10 seconds since there is nothing going on
L = find(t > 10);
H = id(1:L);
X = F(1:L);
theta = inv(H'*H)*H'*X;
Kd = theta;
Ffit = Kd*id;

plottool(1,'F',12,'T (sec)','F');
plot(t,F,'LineWidth',2)
hold on
plot(t,Ffit,'r--','LineWidth',2)

%%%Phase Plot
plottool(1,'phiF',12,'Id','F');
plot(id,F,'LineWidth',2)
hold on
plot(id,Ffit,'r--','LineWidth',2)


%%%Sort id so that it is easier to view
[idsort,I] = sort(id);
Fsort = F(I);

plottool(1,'phiF',12,'Id','F');
plot(idsort,Fsort,'LineWidth',2)
hold on