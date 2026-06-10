if timestepD == 0.01
A =[1.0000         0         0    0.0100         0   -0.0005         0    0.0100         0    0.0006         0   -0.0000         0;
         0    1.0000         0         0   -0.0108         0    0.2022         0    0.0100         0    0.0000         0    0.0000;
         0         0    1.0000         0         0   -0.2022         0   -0.0006         0    0.0096         0   -0.0030         0;
         0         0         0    1.0000         0         0         0         0         0         0         0         0         0;
         0         0         0         0    1.0000         0         0         0   -0.0000         0    0.0079         0    0.0007;
         0         0         0         0         0    1.0000         0    0.0000         0   -0.0001         0    0.0033         0;
         0         0         0         0    0.0000         0    1.0000         0    0.0000         0   -0.0000         0    0.0100;
         0         0         0         0         0   -0.0979         0    0.9993         0    0.0062         0    0.0187         0;
         0         0         0         0    0.0978         0         0         0    0.9970         0    0.0109         0   -0.2004;
         0         0         0         0         0   -0.0050         0   -0.0065         0    0.9344         0   -0.3501         0;
         0         0         0         0   -0.0001         0         0         0   -0.0015         0    0.6144         0    0.0240;
         0         0         0         0         0   -0.0001         0    0.0016         0   -0.0078         0    0.0600         0;
         0         0         0         0    0.0000         0         0         0    0.0002         0   -0.0023         0    0.9896];
B = [    0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
         0         0         0;
   -0.0185         0    0.0136;
         0    0.0011         0;
    0.0597         0   -0.0000;
         0   -0.2134         0;
   -0.1535         0    0.0000;
         0   -0.0077         0];
end
N = 13; %%No states
P = 3; %%No outputs
M = 3; %%No controls
HP = 3; %%Look ahead
C = [zeros(P,N)];
C(1,8) = 1; %U
C(2,11) = 1; %P
C(3,12) = 1; %Q
Q = eye(P*HP);
upenalty = 15;
ppenalty = 10;
qpenalty = 10;
for ii = 1:P
  Q((ii-1)*P+1,(ii-1)*P+1) = upenalty;
  Q((ii-1)*P+2,(ii-1)*P+2) = ppenalty;
  Q((ii-1)*P+3,(ii-1)*P+3) = qpenalty;
end
R = eye(M*HP);
dTpenalty = 2.5;
dapenalty = 0.001;
depenalty = 0.06;
for ii = 1:M
  R((ii-1)*M+1,(ii-1)*M+1) = depenalty;
  R((ii-1)*M+2,(ii-1)*M+2) = dapenalty;
  R((ii-1)*M+3,(ii-1)*M+3) = dTpenalty;
end
KCA = zeros(HP*P,N);
KCAB = zeros(P*HP,M*HP);
U = zeros(HP*M,1);
prodA = A;
for nn = 1:HP
  %%%KCA
  if nn > 1
    prodA = A*prodA;
  end
  tempC = C*prodA;
  KCA(P*(nn-1)+1:P*(nn-1)+P,:) = C*prodA;
  %%%KCAB
  rowKCAB = zeros(P,M*HP);
  for ii = 0:nn-1
    rowcolKCAB = C;
    for jj = 1:nn-ii-1
      rowcolKCAB = rowcolKCAB*A;
    end
    rowcolKCAB = rowcolKCAB*B;
    rowKCAB(:,P*(ii)+1:P*ii+M) = rowcolKCAB;
  end
  KCAB(P*(nn-1)+1:P*(nn-1)+P,:) = rowKCAB;
end
K = inv(KCAB'*Q*KCAB+R)*KCAB'*Q;
