purge

syms x0 y0 xdot0 ydot0 psi0 psi1 x1 y1 xdot1 ydot1 delspan

l0bar = [x0;y0] + delspan*[-sin(psi0);cos(psi0)]
l0hat = [cos(psi0);sin(psi0)]

l1bar = [x1;y1]
l1hat = [-sin(psi1);cos(psi1)]

b = [l1bar - l0bar]
A = [l0hat -l1hat]

lambda = inv(A)*b

lambda = simplify(lambda)

lambda1 = lambda(2)

%%RESULT
x0 = 0;
y0 = 0;
x1 = -50;
y1 = 0;
psi0 = 0.0;
psi1 = 0.1;
delspan = 2.04;

lambda1 = (delspan + y0*cos(psi0) - y1*cos(psi0) - x0*sin(psi0) + x1*sin(psi0))/cos(psi0 - psi1)
