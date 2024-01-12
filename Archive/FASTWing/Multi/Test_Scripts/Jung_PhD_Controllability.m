%%%%Compute the Controllability Gramian for the Linear Longitudinal
%and Lateral matrices for Jung's Phd thesis.

clear
clc
close all


Along = [-7.1264,0.9497,-0.0485,0.0;-122.8676,-4.292,-0.0261,0.0;4.512,-0.0398,-0.1935,-9.81;0,1,0,0];

Blong = [-0.2953,-0.0174;-83.4069,0.8221;-1.3732,6.1044;0.0,0.0];

Alat = [-0.3301 0.4897 0.0570 -0.9939;0 0 1 0.0570;-35.4688 0 -16.8528 0.1524;2.7315 0 -1.1105 -0.5340];

Blat = [0.0,0.1494;0.0,0.0;-182.2395,9.2613;-9.4423,-7.7841];

LongWc = controllability(Along,Blong);

rank(LongWc)

LatWc = controllability(Alat,Blat);

rank(LatWc)

%%%Rank of both of these matrices is 4 which means the total rank
%is 8 which is what we're getting in the system above. 


