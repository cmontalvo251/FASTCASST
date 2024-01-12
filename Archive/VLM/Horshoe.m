function w = Horshoe(P,A,B)
%%This will give the downwash coefficient at a point P given
%%by a horshoe vortex with main filament AB

w = 0;

%%%%%%%%%%%%AB CONTRIBUTION%%%%%%%%

rp = P(1)-A(1);
theta1 = atan2(rp,P(2)-A(2));
theta2 = atan2(rp,P(2)-B(2));
wab = 1/(4*pi*rp)*(cos(theta1)-cos(theta2));

%%%%%%%%%%%%Ainf CONTRIBUTION%%%%%%

rp = P(2)-A(2);
theta1 = atan2(rp,P(1)-A(1));
theta2 = pi;
wainf = 1/(4*pi*rp)*(cos(theta1)-cos(theta2));

%%%%%%%%%%%Binf CONTRIBUTION%%%%%%%

rp = B(2)-P(2);
theta1 = atan2(rp,P(1)-A(1));
theta2 = pi;
wbinf = 1/(4*pi*rp)*(cos(theta1)-cos(theta2));

%%Sum
w = wab + wainf + wbinf;

