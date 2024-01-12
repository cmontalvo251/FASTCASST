function VC_I = Horseshoe(rC_I,rA_I,rB_I,VP_I,G)
%%This will give the downwash coefficient at a point C given
%%by a horshoe vortex with main filament AB, Velocity vector V and
%circulation strength G

%VP_I(3) = 0;

coeff = 1/(4*pi);
v1 = VP_I/norm(VP_I);
%%%%%%%%%%%%AB CONTRIBUTION%%%%%%%%

rAB_I = rB_I-rA_I;
AB = norm(rAB_I);
p2 = rAB_I/AB;
rAC_I = rC_I-rA_I;
p1prime = rAC_I-(rAC_I'*p2)*p2;
p1 = p1prime/norm(p1prime);
p3 = cross(p1,p2);
R = [p1,p2,p3];
rAC_H = R'*rAC_I;
%%rAC_H(3) should be zero
theta1 = atan2(rAC_H(1),rAC_H(2));
theta2 = atan2(rAC_H(1),-AB+rAC_H(2));
wab_H = -(coeff/rAC_H(1))*(cos(theta1)-cos(theta2));
Vab_H = [0;0;wab_H];
Vab_I = R*Vab_H;

%%%%%%%%%%%%Ainf CONTRIBUTION%%%%%%

%%In this vortex p2 = v1
p2 = -v1;
p1prime = rAC_I-(rAC_I'*p2)*p2;
p1 = p1prime/norm(p1prime);
p3 = cross(p1,p2);
R = [p1,p2,p3];
rAC_H = R'*rAC_I;
%%rAC_H(3) should be zero
theta1 = atan2(rAC_H(1),rAC_H(2));
theta2 = pi;
wainf_H = (coeff/rAC_H(1))*(cos(theta1)-cos(theta2));
Vainf_H = [0;0;wainf_H];
Vainf_I = R*Vainf_H;

%%%%%%%%%%%Binf CONTRIBUTION%%%%%%%

%%In this vortex p2 = v1
p2 = -v1;
rBC_I = rC_I-rB_I;
p1prime = rBC_I-(rBC_I'*p2)*p2;
p1 = p1prime/norm(p1prime);
p3 = cross(p1,p2);
R = [p1,p2,p3];
rBC_H = R'*rBC_I;
%%rBC_H(3) should be zero
theta1 = atan2(rBC_H(1),rBC_H(2));
theta2 = pi;
wbinf_H = -(coeff/rBC_H(1))*(cos(theta1)-cos(theta2));
Vbinf_H = [0;0;wbinf_H];
Vbinf_I = R*Vbinf_H;

%%Sum
VC_I = Vab_I + Vainf_I + Vbinf_I;
