if timestepD == 0.001
    A = [
	1.0000         0         0         0         0    0.0010         0         0;
	0    1.0000         0         0    0.0200         0         0    0.0000;
	0         0    1.0000   -0.0200         0         0   -0.0000         0;
	0         0         0    1.0000         0         0    0.0009         0;
	0         0         0         0    1.0000         0         0    0.0010;
	0         0         0         0         0    0.9999         0         0;
	0         0         0         0         0         0    0.7521         0;
	0         0         0         0         0         0         0    0.9990];
    B = [
	0                         0                         0;
	0                         0                         0;
	0                         0                         0;
	0                         0                         0;
	0                         0                         0;
	0                         0       0.00999959909175047;
	-0.0402849133101673                         0                         0;
	0      0.000131337798485395                         0];
  elseif timestepD == 0.01
    A =[
	1.0000         0         0         0         0    0.0100         0         0;
	0    1.0000         0         0    0.2000         0         0    0.0010;
	0         0    1.0000   -0.2000         0         0   -0.0005         0;
	0         0         0    1.0000         0         0    0.0033         0;
	0         0         0         0    1.0000         0         0    0.0099;
	0         0         0         0         0    0.9992         0         0;
	0         0         0         0         0         0    0.0579         0;
	0         0         0         0         0         0         0    0.9897];
    B =[
	0         0         0;
	0         0         0;
	0         0         0;
	0         0         0;
	0         0         0;
	0         0    0.1000;
	-0.1531         0         0;
	0    0.0013         0];
  elseif timestepD == 0.1
    A = [
	1.0000         0         0         0         0    0.0996         0         0;
	0    1.0000         0         0    2.0000         0         0    0.0966;
	0         0    1.0000   -2.0000         0         0   -0.0068         0;
	0         0         0    1.0000         0         0    0.0035         0;
	0         0         0         0    1.0000         0         0    0.0950;
	0         0         0         0         0    0.9920         0         0;
	0         0         0         0         0         0    0.0000         0;
	0         0         0         0         0         0         0    0.9016];

    B = [
	0         0         0;
	0         0         0;
	0         0         0;
	0         0         0;
	0         0         0;
	0         0    0.9960;
	-0.1625         0         0;
	0    0.0125         0];
  elseif timestepD == 1
    A =[
	1.0000         0         0         0         0    0.9610         0         0;
	0    1.0000         0         0   20.0000         0         0    7.2831;
	0         0    1.0000  -20.0000         0         0   -0.0699         0;
	0         0         0    1.0000         0         0    0.0035         0;
	0         0         0         0    1.0000         0         0    0.6227;
	0         0         0         0         0    0.9229         0         0;
	0         0         0         0         0         0    0.0000         0;
	0         0         0         0         0         0         0    0.3548];
    B = [
	0         0         0;
	0         0         0;
	0         0         0;
	0         0         0;
	0         0         0;
	0         0    9.6096;
	-0.1625         0         0;
	0    0.0818         0];
  end
  N = 8; %%No states
  P = 3; %%No outputs
  M = 3; %%No controls
  HP = 100; %%Look ahead
  C = [zeros(P,N-P),eye(P)];
  Q = eye(P*HP);
  upenalty = 15;
  rpenalty = 10;
  qpenalty = 10;
  for ii = 1:P
    Q((ii-1)*P+1,(ii-1)*P+1) = upenalty;
    Q((ii-1)*P+2,(ii-1)*P+2) = rpenalty;
    Q((ii-1)*P+3,(ii-1)*P+3) = qpenalty;
  end
  R = eye(M*HP);
  dTpenalty = 2.5;
  drpenalty = 0.001;
  depenalty = 0.06;
  for ii = 1:M
    R((ii-1)*M+1,(ii-1)*M+1) = depenalty;
    R((ii-1)*M+2,(ii-1)*M+2) = drpenalty;
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
