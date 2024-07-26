function dxdt = Spring(t,x)
global u A B D

pos = x(1);
vel = x(2);

dxdt = A*x+B*u+D;