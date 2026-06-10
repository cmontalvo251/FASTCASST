function [Ts,Fs] = ComputeCollisionForces(xstate)

apprentice_properties

%%%Compute The location of wingtip 1
rcgwt1_B = [0;b/2;0];
rcgwt2_B = [0;-b/2;0];

rcgwt = [rcgwt1_B,rcgwt2_B];

xyz1 = xstate(1:3,1);
xyz2 = xstate(1:3,2);

%%%First Aircraft
T321_all = zeros(3,3,2);
Vwt = zeros(3,2);
for n = 1:2
    phi = xstate(4,n);
    theta = xstate(5,n);
    psi = xstate(6,n);
    T3 = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];
    T2 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    T1 = [1 0 0;0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    T321 = T3*T2*T1;
    T321_all(:,:,n) = T321;
    uvw = xstate(7:9,n);
    xyzdot = T321*uvw;
    pqr = xstate(10:12,n);
    p = pqr(1);
    q = pqr(2);
    r = pqr(3);
    pqrskew = [0 -r q;r 0 -p;-q p 0];
    Vwt(:,n) = xyzdot + pqrskew*rcgwt(:,n);
end

rwt1 = xyz1 + T321_all(:,:,1) * rcgwt1_B;
rwt2 = xyz2 + T321_all(:,:,2) * rcgwt2_B;

delwt = rwt2 - rwt1;
delwtdot = Vwt(:,2) - Vwt(:,1);

kx = 1.5;
ky = 5.0;
kz = 1.5;
K = [kx 0 0;0 ky 0;0 0 kz];
C = 0.5*kx*eye(3);

Fs = K*delwt + C*delwtdot;

delptp = xstate(4:6,2) - xstate(4:6,1);
delpqr = xstate(10:12,2) - xstate(10:12,1);

krot = 2*eye(3);
crot = 1*eye(3);

Ts = krot*delptp + crot*delpqr;

if abs(delptp(1)) > 20*pi/180
    Ts(1) = 0;
end
%if sum(abs(Fs)) > 0
%    eqfdfds = 1;
%end
Ts = Ts + cross(rcgwt(:,1),Fs);

%Ts = 0*Ts;
%Fs = 0*Fs;
