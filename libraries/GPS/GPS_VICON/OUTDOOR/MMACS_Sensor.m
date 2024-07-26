purge
%%Ok so let's assume we have 4 markers on the aircraft

update = 1000; %Hz
pts = 1000;
tend = pts/update;
time = linspace(0,tend,pts);
N=100;

%%Create Data Vectors
cgVec = [time',ones(pts,1),ones(pts,1)];
ptpVec = [ones(pts,1),ones(pts,1),ones(pts,1)];

%%Now add noise to cgVec
NoisyGPS = (GPSsim(cgVec',0))';
NoisyPTP = (HorizonSim(ptpVec',0))';

noisyData = [NoisyGPS,NoisyPTP];

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
figure()
plot(time,stateHistory(:,1),'k','LineWidth',2)
hold on
plot(time,noisyData(:,1),'g-','LineWidth',2)
grid on
ylabel('x-Pos [m]','FontSize',14)
figure()
plot(time,stateHistory(:,2),'k','LineWidth',2)
hold on
plot(time,noisyData(:,2),'g-','LineWidth',2)
grid on
ylabel('y-Pos [m]','FontSize',14)
figure()
plot(time,stateHistory(:,3),'k','LineWidth',2)
hold on
plot(time,noisyData(:,3),'g-','LineWidth',2)
grid on
ylabel('z-Pos [m]','FontSize',14)
xlabel('Time [s]','FontSize',14)
figure()
plot(time,stateHistory(:,4)*180/pi,'k','LineWidth',2)
hold on
plot(time,noisyData(:,4)*180/pi,'g-','LineWidth',2)
plot(time,ptpVec(:,1)*180/pi,'g-','LineWidth',2)
grid on
ylabel('Roll [deg]','FontSize',14)
figure()
plot(time,stateHistory(:,5)*180/pi,'k','LineWidth',2)
hold on
plot(time,noisyData(:,5)*180/pi,'g-','LineWidth',2)
grid on
ylabel('Pitch [deg]','FontSize',14)
figure()
plot(time,stateHistory(:,6)*180/pi,'k','LineWidth',2)
hold on
plot(time,noisyData(:,6)*180/pi,'g-','LineWidth',2)
grid on
ylabel('Yaw [deg]','FontSize',14)
xlabel('Time [s]','FontSize',14)

figure()
plot(time,stateDotHistoryFilt(:,1),'k','LineWidth',2)
grid on
ylabel('x-dot [m/s]','FontSize',14)
figure()
plot(time,stateDotHistoryFilt(:,2),'k','LineWidth',2)
grid on
ylabel('y-dot [m/s]','FontSize',14)
figure()
plot(time,stateDotHistoryFilt(:,3),'k','LineWidth',2)
grid on
ylabel('z-dot [m/s]','FontSize',14)
xlabel('Time [s]','FontSize',14)
figure()
plot(time,stateDotHistoryFilt(:,4),'k','LineWidth',2)
grid on
ylabel('roll-dot [rad/s]','FontSize',14)
figure()
plot(time,stateDotHistoryFilt(:,5),'k','LineWidth',2)
grid on
ylabel('pitch-dot [rad/s]','FontSize',14)
figure()
plot(time,stateDotHistoryFilt(:,6),'k','LineWidth',2)
grid on
ylabel('yaw-dot [rad/s]','FontSize',14)
xlabel('Time [s]','FontSize',14)