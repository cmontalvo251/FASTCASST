purge

%%Get A and B matrices
global HP P M N K KCAB KCA timestepD A B
global UCOMMAND ZCOMMAND MSLOPE BINTERCEPT ARCLENGTHPARENT
global nominal control0
N = 13; %%No states
P = 3; %%No outputs
M = 3; %%No controls
HP = 3; %%Look ahead
UCOMMAND = 20;
ZCOMMAND = -200;
MSLOPE = 0;
BINTERCEPT = 0;
ARCLENGTHPARENT = 10;
t = 0;
timestep = 0.001;
timestepD = 0.01;
state = zeros(13,1);
nominal = state;
e = 0;
control0 = [0;0;-0.06103;0];
[state_dot,control] = LinearM(t,state,e);
%%Add a state(zint)
A = [[A;zeros(1,13)],zeros(14,1)];
A(14,3) = 1;
%%Add another state(yint)
A = [[A;zeros(1,14)],zeros(15,1)];
A(15,2) = 1;
B = [B;zeros(2,3)];

%%Open Loop Eigen Values
s = eig(A)

%%Closed Loop Eigen Values Child Aircraft(A+BK)
Kptheta1 = -5;
Kptheta = 1;
Kdtheta = 0.06;
KItheta = 0.02;
Kpphi = 3;
Kdphi = 0.2;
Kppsi = -3;
Kpy = -0.004;
Kdy = 0.03;
KIy = -0.0003;
K = [0,0,Kptheta1*Kptheta,0,0,Kptheta1*Kdtheta*A(3,6),0,Kptheta1*Kdtheta*A(3,8),0,Kptheta1*Kdtheta*A(3,10),0,0,0,Kptheta1*KItheta,0;0,-Kpphi*Kppsi*Kpy,0,0,Kpphi,0,0,0,-Kpphi*Kppsi*Kdy,0,Kdphi,0,0,0,-Kpphi*Kppsi*KIy;zeros(1,15)];
Acl = A+B*K;

scl = eig(Acl)