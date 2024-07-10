purge
%%Ok so let's assume we have 4 markers on the aircraft

update = 1000; %Hz
pts = 1000;
tend = pts/update;
time = linspace(0,tend,pts);
N=100;

%%True Data
cg = [0;0;0];
b = 0.432;
marker1B = [1;0;0]*b;
marker2B = [-1;0;0]*b;
marker3B = [0;1;0]*b;
marker4B = [0;-1;0]*b;
%%Create Data Vectors
cgVec = ones(pts,1)*cg';
cgVec(:,1) = time.*17;
ptpVec = 0.*cgVec;
for ii = 1:pts
  ptp = [sin(N*time(ii));0.0;0];
  ptpVec(ii,:) = ptp;
  R = R123(ptp(1),ptp(2),ptp(3));
  marker1 = R*marker1B;
  marker2 = R*marker2B;
  marker3 = R*marker3B;
  marker4 = R*marker4B;
  mark1Vec(ii,:) = cgVec(ii,:) + marker1';
  mark2Vec(ii,:) = cgVec(ii,:) + marker2';
  mark3Vec(ii,:) = cgVec(ii,:) + marker3';
  mark4Vec(ii,:) = cgVec(ii,:) + marker4';
end

%%Now add noise to all markers
mark1N = GPSsim(mark1Vec',0)';
mark2N = GPSsim(mark2Vec',0)';
mark3N = GPSsim(mark3Vec',0)';
mark4N = GPSsim(mark4Vec',0)';
mark = zeros(pts,3,4);
mark(:,:,1) = mark1N;
mark(:,:,2) = mark2N;
mark(:,:,3) = mark3N;
mark(:,:,4) = mark4N;

% %%plot raw data from markers
% for ii = 1:3
%   plottool(1,'XYZ',12,'Time(sec)','Time(sec)','State(mm)');
%   for jj = 1:4
%     subplot(2,2,jj)
%     plot(tspan,mark(:,ii,jj))
%     hold on
%   end
% end
%%Now pull out cg and ptp from the 4 markers
noisyData = zeros(pts,6);
for ii = 1:pts
  noisyData(ii,1:3) = (1/4)*(mark1N(ii,:)+mark2N(ii,:)+mark3N(ii,:)+mark4N(ii,:));
  %%Get the xbar vector
  xbar = mark1N(ii,:)-mark2N(ii,:);
  xhat = xbar/norm(xbar);
  %%Get the ybar vector
  ybar = mark3N(ii,:)-mark4N(ii,:);
  yhat = ybar/norm(ybar);
  %%Zbar is just x cross y
  zhat = cross(xhat,yhat);
  %%Contruct R matrix
  R = [xhat;yhat;zhat];
  %%Pull out phi theta psi from R matrix
  ptp = RPTP(R);
  noisyData(ii,4:6) = ptp';
end

%%Filter the data to get derivatives
stateHistory = [cgVec ptpVec];
currentStateDotFilt = zeros(1,6);
for i = 1:pts
    if i > 2 % need 2 previous measurements to get estimate
        % in the lab we use 2 different filters for position (slower outer
        % loop) and attitude (fast inner loop) you do not have to do it
        % this way
        % 2nd order low pass discrete filters use current and past 2 measurment states (i-2:i) and past 2 filtered derivative states (i-2:i-1) to get current
        % derivative states
        currentPosDotFilt = dotFilterOuterLoop(stateDotHistoryFilt(i-2:i-1,1:3),noisyData(i-2:i,1:3));
        currentAttDotFilt = dotFilterInnerLoop(stateDotHistoryFilt(i-2:i-1,4:6),noisyData(i-2:i,4:6));
        currentStateDotFilt = [currentPosDotFilt currentAttDotFilt];
    end
    stateDotHistoryFilt(i,:) = currentStateDotFilt;
end

%%Now plot xyz and ptp
% figure()
% plot(time,stateHistory(:,1),'k','LineWidth',2)
% hold on
% plot(time,noisyData(:,1),'g-','LineWidth',2)
% grid on
% ylabel('x-Pos [m]','FontSize',14)
% figure()
% plot(time,stateHistory(:,2),'k','LineWidth',2)
% hold on
% plot(time,noisyData(:,2),'g-','LineWidth',2)
% grid on
% ylabel('y-Pos [m]','FontSize',14)
% figure()
% plot(time,stateHistory(:,3),'k','LineWidth',2)
% hold on
% plot(time,noisyData(:,3),'g-','LineWidth',2)
% grid on
% ylabel('z-Pos [m]','FontSize',14)
% xlabel('Time [s]','FontSize',14)
figure()
plot(time,stateHistory(:,4)*180/pi,'k','LineWidth',2)
hold on
plot(time,noisyData(:,4)*180/pi,'g-','LineWidth',2)
plot(time,ptpVec(:,1)*180/pi,'g-','LineWidth',2)
grid on
ylabel('Roll [deg]','FontSize',14)
% figure()
% plot(time,stateHistory(:,5)*180/pi,'k','LineWidth',2)
% hold on
% plot(time,noisyData(:,5)*180/pi,'g-','LineWidth',2)
% grid on
% ylabel('Pitch [deg]','FontSize',14)
% figure()
% plot(time,stateHistory(:,6)*180/pi,'k','LineWidth',2)
% hold on
% plot(time,noisyData(:,6)*180/pi,'g-','LineWidth',2)
% grid on
% ylabel('Yaw [deg]','FontSize',14)
% xlabel('Time [s]','FontSize',14)

% figure()
% plot(time,stateDotHistoryFilt(:,1),'k','LineWidth',2)
% grid on
% ylabel('x-dot [m/s]','FontSize',14)
% figure()
% plot(time,stateDotHistoryFilt(:,2),'k','LineWidth',2)
% grid on
% ylabel('y-dot [m/s]','FontSize',14)
% figure()
% plot(time,stateDotHistoryFilt(:,3),'k','LineWidth',2)
% grid on
% ylabel('z-dot [m/s]','FontSize',14)
% xlabel('Time [s]','FontSize',14)
figure()
plot(time,stateDotHistoryFilt(:,4),'k','LineWidth',2)
hold on
plot(time,N*cos(N*time),'k--','LineWidth',2)
grid on
ylabel('roll-dot [rad/s]','FontSize',14)
% figure()
% plot(time,stateDotHistoryFilt(:,5),'k','LineWidth',2)
% grid on
% ylabel('pitch-dot [rad/s]','FontSize',14)
% figure()
% plot(time,stateDotHistoryFilt(:,6),'k','LineWidth',2)
% grid on
% ylabel('yaw-dot [rad/s]','FontSize',14)
% xlabel('Time [s]','FontSize',14)