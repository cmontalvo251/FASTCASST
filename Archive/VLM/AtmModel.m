function Vatm_I = AtmModel(xyz)
%%%compute the inertial components of the atmospheric winds as a
%function of xyz which are in inertial coordinates

xI = xyz(1);
yI = xyz(2);
zI = xyz(3);

Vatm_I = [0;0;0];