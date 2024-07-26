function [rho Vw] = atm_update(t,x,dt,den_calc,VwTAB,turb_level,b)
%atmosphere model
%inputs
%t          = time
%x          = state vector
%dt         = time step between gust updates
%den_calc   = 0=sea level density, 1=standard atmosphere
%VwTAB      = mean wind profile
%             VwTAB = vector: [North East Down] for constant wind
%             VwTAB = table: [Altitude North East Down] for specified wind profile
%turb_level = standard deviation of vertical wind component in m/s
%b          = reference span in meters
%outputs
%rho = density (kg/m^3)
%Vw = [V_north; V_east; V_down; roll; pitch; yaw] (m/s) or (rad/s)

persistent Vwg   %save gust for dynamic turbulence model
if isempty(Vwg), Vwg = randn(6,1)*turb_level; end

h = -x(3);
ptp = x(4:6);
uvw = x(7:9);

%atmospheric density
if den_calc == 0
    rho = 1.225;
else
    rho = 1.225*(1-h/.3048/145442).^4.255876;
end

%wind interpolation
if numel(VwTAB,1) < 8
    Vw = VwTAB(1:3)';
else
    Vw = interp1(VwTAB(:,1),VwTAB(2:4),h,'cubic')';
end

%Dryden Turbulence Model
%from MIL-STD-1797A
T_BI = L1(ptp(1))*L2(ptp(2))*L3(ptp(3));
V = norm(T_BI*uvw - [Vw(1);Vw(2);Vw(3)]); %airspeed in canopy frame

if h < 10, h=10; end

Lu = h/(.177+.000823*h/.3048)^1.2;
Lv = Lu;
Lw = h;

sigw = turb_level;
sigu = sigw/(.177+.000823*h/.3048)^0.4;
sigv = sigu;

ugo = Vwg(1);
vgo = Vwg(2);
wgo = Vwg(3);
pgo = Vwg(4);
qgo = Vwg(5);
rgo = Vwg(6);
ug = (1-V/Lu*dt)*ugo+sqrt(2*V/Lu*dt)*sigu*randn;
vg = (1-V/Lv*dt)*vgo+sqrt(2*V/Lv*dt)*sigv*randn;
wg = (1-V/Lw*dt)*wgo+sqrt(2*V/Lw*dt)*sigw*randn;
pg = (1-2.6/sqrt(Lw*b)*dt)*pgo+sqrt(2*2.6/sqrt(Lw*b)*dt)*1.9/sqrt(2*Lw*b)*sigw*randn;
qg = (1-pi*V/(4*b)*dt)*qgo+pi/(4*b)*(wg-wgo);
rg = (1-pi*V/(3*b)*dt)*rgo+pi/(3*b)*(vg-vgo);

Vwg= [ug vg wg pg qg rg]';

Vw = [Vw(1) + ug;...
      Vw(2) + vg;...
      Vw(3) + wg; ...
     [pg;qg;rg]];