function MMOI = MassMOI()
% Moment of Inertia
% a is along x, b is along y, l is along z
% all units in meters and kg
% Use rectangular approximations

awing = .224;                 % Main wing
bwing = 1.5;
lwing = .016;
mwing = .35;
d_cog_wing = [.012,0,.11];   % Distance from composite center of mass [x y z]

abody = .95;               % Body
bbody = .09;
lbody = .096;
mbody = 1.089;
d_cog_body = [.04 0 .01];

ahtail = .13167;               % Horizontal tail surface
bhtail = .56;
lhtail = .0045;
mhtail = .05;
d_cog_htail = [.575 0 .016];

avtail = .5833;                % Vertctical tail surface
bvtail = .0045;
lvtail = .085;
mvtail = .035;
d_cog_vtail = [.575 0 .07];

%Inertia = I(length,width,thickness,mass);
Iwing = I(awing,bwing,lwing,mwing);
%Parallel Axis Thereom = Ip(Inertia,distance in the [X Y Z],mass);
Ipwing = Ip(Iwing,d_cog_wing,mwing);

Ibody = I(abody,bbody,lbody,mbody);
Ipbody = Ip(Ibody,d_cog_body,mbody);

Ihtail = I(ahtail,bhtail,lhtail,mhtail);
Iphtail = Ip(Ihtail,d_cog_htail,mhtail);

Ivtail = I(avtail,bvtail,lvtail,mvtail);
Ipvtail = Ip(Ivtail,d_cog_vtail,mvtail);

MMOI = Ipwing + Ipbody + Iphtail + Ipvtail;

function MOI = I(a,b,l,m)

Ixx = m*(b^2 + l^2)/12;
Iyy = m*(a^2 + l^2)/12;
Izz = m*(a^2 + b^2)/12;

MOI = [Ixx,Iyy,Izz];

function PAT = Ip(MOI,D_COG,m)

Ixxp = MOI(1) + m*norm([D_COG(2) D_COG(3)])^2;
Iyyp = MOI(2) + m*norm([D_COG(1) D_COG(3)])^2;
Izzp = MOI(3) + m*norm([D_COG(1) D_COG(2)])^2;

PAT = [Ixxp,Iyyp,Izzp];
