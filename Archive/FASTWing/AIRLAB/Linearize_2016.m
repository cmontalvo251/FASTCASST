%clear
clc
close all

xeq = [0 0 -200 0 0 0 20 0 0 0 0 0]'; %[x y z phi theta psi u v w p q r]
A = zeros(12,12);

feq = Derivatives(0,xeq)
xm =xeq*ones(1,12);

d = 1e-6;

dx = diag(d*ones(12,1));
xepdx = xm + dx;
xemdx = xm - dx;

for col = 1:12
    A(:,col) = (Derivatives(0,xepdx(:,col))-Derivatives(0,xemdx(:,col)))/(2*d);
end
[v,s]=eig(A)
diag(s)