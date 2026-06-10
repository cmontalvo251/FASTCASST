purge
%%%Nominal states
nominal = [0 0 -200 0 0 0 20 0 0 0 0 0]';
controls = [0;0;0;0];
%Linear Model is just
%order = [1 2 3 5 6 7 11 12];
order = 1:12;
l = length(order);
%x y z dtheta dpsi u dq dr
%[de dr dT]
A = zeros(l,l);
B = zeros(l,3);
del = 1e-8;
%%Get nominal derivatives
Ffull0 = ACdot(0,nominal,0);
F0 = Ffull0(order);
%%Strip states we want
counter = 1;
for ii = order
  %%perterb iith state
  perterb = nominal;
  perterb(ii) = perterb(ii) + del;
  Ffull = ACDot(0,perterb,0);
  F = Ffull(order);
  A(:,counter) = (F - F0)/del;
  counter = counter + 1;
end
%%Now we need the controls perturbed
counter = 1;
for ii = [3 2 4]
  perterb = controls;
  perterb(ii) = perterb(ii) + del;
  Ffull = ACdot(0,nominal,perterb);
  F = Ffull(order);
  B(:,counter) = (F-F0)/del;
  counter = counter + 1;
end
A
B
NLinear = 1;
timestepDiscrete = 0.001;
Ad = expm(A*timestepDiscrete);
jump = 1000;
Adjump = Ad^jump
Ap = A(6:8,6:8);
Adp = Ad(6:8,6:8);
Bp = B(6:8,:);
Bdp = inv(Ap)*(Adp-eye(3))*Bp;
Bdpjump = zeros(3,3);
for ii = 1:jump
  Bdpjump = Bdpjump + Adp^(ii-1)*Bdp;
end
Bdjump = [zeros(5,3);Bdpjump]
