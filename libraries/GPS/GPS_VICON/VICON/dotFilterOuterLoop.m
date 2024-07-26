function filt = dotFilterOuterLoop(oldFilt, val)
% discrete derivative filter for position states
% filter values calculated by designing a 2nd order continous domain
% transfer function then using MATLAB continous to discrete function,
% c2d(), to perfrom a tustin transformation

% transfer function choosen to given faster filtered derivated for
% inner loop states making experimental control design easier in VICON

%***note the discrete filter is time step dependent, this filter was
%calculated for timestep = 1/120 = 0.0083333

% Continous transfer function
%           16 s
% T(s) =   ------
%          s + 16
% c2d(tf([16 0],[1 16]),1/120,'tustin')
%  
%       Transfer function:
%       15 z - 15
%       ---------
%       z - 0.875
%  
% Sampling time: 0.0083333
% Inputs - oldFilt: previous 2 time step filtered values, 
%              oldFilt = [filt(i-2,:),filt(i-1,:)]
%          val: input state to be differentiated, current values and
%          previous 2 time steps
%              val = [input(i-2,:),input(i-1,:), input(i,:)]
% Outputs - filt: filetered value at current time step

%tau is 1/16 now. We want it faster so maybe 1/100?

%filt = ([0 -99.5 99.5]*val + [0 0.99]*oldFilt);
filt = ([0 -95.24 95.24]*val + [0 0.9048]*oldFilt);
%filt = [0 -0.004975 0.004975]*val + [0 0.99]*oldFilt;




