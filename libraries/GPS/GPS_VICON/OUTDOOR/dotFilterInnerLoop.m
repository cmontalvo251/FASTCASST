function filt = dotFilterInnerLoop(oldFilt, val)
% discrete derivative filter for attitude states
% filter values calculated by designing a 2nd order continous domain
% transfer function then using MATLAB continous to discrete function,
% c2d(), to perfrom a tustin transformation

% transfer function choosen to given faster filtered derivated for
% inner loop states making experimental control design easier in VICON

%***note the discrete filter is time step dependent, this filter was
%calculated for timestep = 1/120 = 0.0083333

% Continous transfer function
%                64*64s
% T(s) = -----------------------
%         s^2 + 2*0.7*64*s + 64^2
% c2d(tf([64*64 0],[1 2*.7*64 64*64]),1/120,'tustin')

%%%%%%%%%%%%%%%%%NOTE%%%%%%%%%%%%%%

%bode(T(s)) and look at the magnitude and phase to double check 
%GET RID OF THE 'S' on the top of the function but actually if you
%do that it is no longer a derivative filter

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  
% Transfer function:
%   11.82 z^2 - 11.82
% ----------------------
% z^2 - 1.286 z + 0.4831
%  
% Sampling time: 0.0083333


% Inputs - oldFilt: previous 2 time step filtered values, 
%              oldFilt = [filt(i-2,:),filt(i-1,:)]
%          val: input state to be differentiated, current values and
%          previous 2 time steps
%              val = [input(i-2,:),input(i-1,:), input(i,:)]
% Outputs - filt: filetered value at current time step


%filt = [-11.82 0 11.82]*val+[-0.4831 1.286]*oldFilt;
p1 = 1.99;
p2 = -0.9902;
p3 = 0.2438*15/4;


p1 = 1.902;
p2 = -0.9067;
p3 = 2.333;
filt = [-p3 0 p3]*val+[p2 p1]*oldFilt;
