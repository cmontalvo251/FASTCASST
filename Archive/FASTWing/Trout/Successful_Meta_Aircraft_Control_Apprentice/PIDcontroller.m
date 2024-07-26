function [de,da,dT,dr] = PIDcontroller(xyz,ptp,uvw,pqr,xyzdot,ptpdot,t)

% de_pid = [-.05 -.05]';

kd_u = -3;
kp_a = 100;
kd_a = 50;
% kp_e = 0.6;
% kd_e = 2;   %1.5; %from arduino
kp_e = 4;
kd_e = 2;
kp_z = .6;
kd_z = .8;

xyz_c = [xyz(1)+15 0 -200]';
uvw_c = [16 0 0]';

% if t < 5
ptp_c = [0*pi/180 0*pi/180 0]';
% else
%     ptp_c = [0*pi/180 0*pi/180 0]';
% end

e_pos = xyz - xyz_c;

e_vel = xyzdot - uvw_c;

theta_c = kp_z*(e_pos(3)) + kd_z*(e_vel(3));

if abs(theta_c) > 30*pi/180
  theta_c = 30*pi/180 * sign(theta_c);
end

da = kp_a*(ptp(1) - ptp_c(1)) + kd_a*(ptpdot(1));
theta_c = 8*pi/180;
de = kp_e*(ptp(2) - theta_c) + kd_e*(ptpdot(2));
% de = kp_e*(e_pos(3)) + kd_e*(uvw(3) - uvw_c(3));

dr = 0;
dT = kd_u*(uvw(1) - uvw_c(1));

%de = 0;
dr = 0;
dT = 0;

