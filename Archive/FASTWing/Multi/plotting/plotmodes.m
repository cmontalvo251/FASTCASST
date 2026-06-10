purge

%%%%Plot Modes
load Modes.mat

%%%%%%%THIS IS FOR K1N%%%%%%%

% pitchend = pitch;
% flappingend = flapping;
% leadlagend = leadlag;

% load Modes_Final.mat

% leadlagfinal = leadlag;

% pitch(1,:) = [];
% pitch(2,:) = [];
% flapping(1,:) = [];
% leadlag(1,:) = [];
% leadlag(1,:) = [];

% pitch = [pitchend;pitch];
% leadlag = [leadlagend;leadlag];
% flapping = [flappingend;flapping];
% rflapping = real(flapping);
% iflapping = abs(imag(flapping));
% flapping(:,1) = rflapping(:,1) + iflapping(:,1)*i;
% flapping(:,2) = rflapping(:,2) - iflapping(:,2)*i;

% rleadlag = real(leadlag);
% ileadlag = abs(imag(leadlag));
% leadlag(:,1) = rleadlag(:,1) + ileadlag(:,1)*i;
% leadlag(:,2) = rleadlag(:,2) - ileadlag(:,2)*i;
% leadlag(8,:) = [-5.098,-5.098];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%FOR C1n%%%%%%%%%%%%%
% leadlag = [leadlag(1:4,:);[-60,-60];leadlag(5:end,:)];
% rflapping = real(flapping);
% iflapping = abs(imag(flapping));
% flapping(:,1) = rflapping(:,1) + iflapping(:,1)*i;
% flapping(:,2) = rflapping(:,2) - iflapping(:,2)*i;

% realend = flapping(9:10,1)';
% imagend = flapping(9:10,2);
% flapping = [flapping(1:8,:);[imagend(1) imagend(1)];[imagend(2) imagend(2)];realend];
% rflapping = real(flapping);
% iflapping = abs(imag(flapping));
% flapping(:,1) = rflapping(:,1) + iflapping(:,1)*i;
% flapping(:,2) = rflapping(:,2) - iflapping(:,2)*i;
% flapping = [flapping(1:10,:);[-11,-11];flapping(11:end,:)];

%%%%%%%%%FOR K1t%%%%%%%%%%%%%%%%%%%%%
% pitchstart = pitch;
% load Modes_Final_K1t.mat

% flapping(5:end,:) = [];

% pitch(1,:) = [];
% pitch(1,:) = [];
% pitch = [pitchstart(1:7,:);[-13.7,-13.7];pitchstart(8:end,:);pitch];
% pitch(1,:) = [];

%%%FOR C1t%%%%%%%%%%%%%%%%%%%%%
pitch = [pitch(1:3,:);[-70,-70];pitch(4:end,:)];

plottool(1,'Pitch',12,'Real','Imaginary');
plot(pitch,'k-x','LineWidth',2)

plottool(1,'Flapping',12,'Real','Imaginary');
plot(flapping,'k-x','LineWidth',2)

plottool(1,'Lead Lag',12,'Real','Imaginary');
plot(leadlag,'k-x','LineWidth',2)

plottool(1,'All',12,'Real','Imaginary');
p1 = plot(pitch,'k-s');
p2 = plot(flapping,'k-x');
p3 = plot(leadlag,'k-d');
legend([p1(1) p2(1) p3(1)],'Pitch Mode','Flapping Mode','LeadLag Mode')