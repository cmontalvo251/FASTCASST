% load in the VICON noise measurement data file and process to get
% derivatives

clc
close all
clear all

% load data file
data = load('MeasData.mat'); % load into struct so don't have random variables and indexes floating around in code
time = data.time; % [frames x 1] vector of timestamp at each measuremnt (timestep = 1/120 s)
stateHistory = data.stateHistory./1000; % [frames x 6] = [ x y z phi theta psi], position is in [mm], euler angles in [rad]

frames = length(time); % total number of measurements taken
% preallocate memory for filter variables

stateDotHistoryFilt = zeros(frames,6);
currentAttDotFilt = zeros(1,3);
currentPosDotFilt = zeros(1,3);
currentStateDotFilt = zeros(1,6);

%%true data
trueData = ones(frames,1)*[12.8  10.3    1   0.0134   -0.0002   -0.0054]./1000;

noisyData = GPSsim(trueData',2)';


% for i = 1:frames
%     if i < 0 % need 2 previous measurements to get estimate
%         % in the lab we use 2 different filters for position (slower outer
%         % loop) and attitude (fast inner loop) you do not have to do it
%         % this way
%         % 2nd order low pass discrete filters use current and past 2 measurment states (i-2:i) and past 2 filtered derivative states (i-2:i-1) to get current
%         % derivative states
%         currentPosDotFilt = dotFilterOuterLoop(stateDotHistoryFilt(i-2:i-1,1:3),stateHistory(i-2:i,1:3));
%         currentAttDotFilt = dotFilterInnerLoop(stateDotHistoryFilt(i-2:i-1,4:6),stateHistory(i-2:i,4:6));
%         currentStateDotFilt = [currentPosDotFilt currentAttDotFilt];
%     end
%     stateDotHistoryFilt(i,:) = currentStateDotFilt;
% end

figure
subplot(3,2,1)
plot(time,stateHistory(:,1),'k','LineWidth',2)
hold on
plot(time,trueData(:,1),'r-','LineWidth',2)
plot(time,noisyData(:,1),'g-','LineWidth',2)
grid on
ylabel('x-Pos [mm]','FontSize',14)
subplot(3,2,3)
plot(time,stateHistory(:,2),'k','LineWidth',2)
hold on
plot(time,trueData(:,2),'r-','LineWidth',2)
plot(time,noisyData(:,2),'g-','LineWidth',2)
grid on
ylabel('y-Pos [mm]','FontSize',14)
subplot(3,2,5)
plot(time,stateHistory(:,3),'k','LineWidth',2)
hold on
plot(time,trueData(:,3),'r-','LineWidth',2)
plot(time,noisyData(:,3),'g-','LineWidth',2)
grid on
ylabel('z-Pos [mm]','FontSize',14)
xlabel('Time [s]','FontSize',14)
subplot(3,2,2)
plot(time,stateHistory(:,4)*180/pi,'k','LineWidth',2)
hold on
plot(time,trueData(:,4)*180/pi,'r-','LineWidth',2)
plot(time,noisyData(:,4)*180/pi,'g-','LineWidth',2)
grid on
ylabel('Roll [deg]','FontSize',14)
subplot(3,2,4)
plot(time,stateHistory(:,5)*180/pi,'k','LineWidth',2)
hold on
plot(time,trueData(:,5)*180/pi,'r-','LineWidth',2)
plot(time,noisyData(:,5)*180/pi,'g-','LineWidth',2)
grid on
ylabel('Pitch [deg]','FontSize',14)
subplot(3,2,6)
plot(time,stateHistory(:,6)*180/pi,'k','LineWidth',2)
hold on
plot(time,trueData(:,6)*180/pi,'r-','LineWidth',2)
plot(time,noisyData(:,6)*180/pi,'g-','LineWidth',2)
grid on
ylabel('Yaw [deg]','FontSize',14)
xlabel('Time [s]','FontSize',14)


figure
subplot(3,2,1)
plot(time,stateDotHistoryFilt(:,1),'k','LineWidth',2)
grid on
ylabel('x-dot [mm/s]','FontSize',14)
subplot(3,2,3)
plot(time,stateDotHistoryFilt(:,2),'k','LineWidth',2)
grid on
ylabel('y-dot [mm/s]','FontSize',14)
subplot(3,2,5)
plot(time,stateDotHistoryFilt(:,3),'k','LineWidth',2)
grid on
ylabel('z-dot [mm/s]','FontSize',14)
xlabel('Time [s]','FontSize',14)
subplot(3,2,2)
plot(time,stateDotHistoryFilt(:,4)*180/pi,'k','LineWidth',2)
grid on
ylabel('roll-dot [deg/s]','FontSize',14)
subplot(3,2,4)
plot(time,stateDotHistoryFilt(:,5)*180/pi,'k','LineWidth',2)
grid on
ylabel('pitch-dot [deg/s]','FontSize',14)
subplot(3,2,6)
plot(time,stateDotHistoryFilt(:,6)*180/pi,'k','LineWidth',2)
grid on
ylabel('yaw-dot [deg/s]','FontSize',14)
xlabel('Time [s]','FontSize',14)