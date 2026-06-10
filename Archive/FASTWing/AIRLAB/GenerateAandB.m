purge

global nominal AEROMODEL

AEROMODEL = 1;
%%%Nominal states
nominal = [0;0;-200;0;0.0560096547756443;0;20.192;0;1.13214709899632;0;0;0];
controls = [0;0;0;0];
%Linear Model is just
order = 1:12;
l = length(order);
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
  Ffull = ACdot(0,perterb,0);
  F = Ffull(order);
  A(:,counter) = (F - F0)/del;
  counter = counter + 1;
end
%%Now we need the controls perturbed
counter = 1;
for ii = [3 1 4]
  perterb = controls;
  perterb(ii) = perterb(ii) + del;
  Ffull = ACdot(0,nominal,perterb);
  F = Ffull(order);
  B(:,counter) = (F-F0)/del;
  counter = counter + 1;
end
%%Now we change the system from dx to just x and augment
%%the state with u0

%%First add a column of zeros
A = [A(:,1:3),zeros(l,1),A(:,4:end)];
%%Now add a row of zeros
A = [A(1:3,:);zeros(1,l+1);A(4:end,:)];
%Finally change A(1,4) = 1 to account for u0
A(1,4) = 1
%%For B we need to simply add a row of zeros
B = [zeros(1,3);B]

timestepDiscrete = 0.001;
Ad = expm(A*timestepDiscrete);
jump = 10;
Adjump = Ad^jump
Ap = A(8:13,8:13);
Adp = Ad(8:13,8:13);
Bp = B(8:13,:);
Bdp = inv(Ap)*(Adp-eye(6))*Bp;
Bdpjump = zeros(6,3);
for ii = 1:jump
  Bdpjump = Bdpjump + Adp^(ii-1)*Bdp;
end
Bdjump = [zeros(7,3);Bdpjump]
