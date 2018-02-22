% Clear workspace
clear all;
clc;

% Read data from text file
gps1 = tdfread('gps.txt',',');    % robot 1
gps2 = tdfread('gps2.txt',',');   % robot 2
gps3 = tdfread('gps3.txt',',');   % ground 1
gps4 = tdfread('gps4.txt',',');   % ground 2

% Set to array [xLat, yLat, alt]
G1 = [gps3.field0x2Elatitude gps3.field0x2Elongitude gps3.field0x2Ealtitude];
G2 = [gps4.field0x2Elatitude gps4.field0x2Elongitude gps4.field0x2Ealtitude];
R1 = [gps1.field0x2Elatitude gps1.field0x2Elongitude gps1.field0x2Ealtitude];
R2 = [gps2.field0x2Elatitude gps2.field0x2Elongitude gps2.field0x2Ealtitude];

% Take average of ground station and robot GPS units
G = [mean([G1(:,1),G2(:,1)],2), mean([G1(:,2),G2(:,2)],2), mean([G1(:,3),G2(:,3)],2)];
R = [mean([R1(:,1),R2(:,1)],2), mean([R1(:,2),R2(:,2)],2), mean([R1(:,3),R2(:,3)],2)];

% Convert from lat-lon to meters
deg2north = 90;
Gm = lla2flat(G,[G(1,1) G(1,2)],deg2north,-G(1,3));
Rm = lla2flat(R,[G(1,1) G(1,2)],deg2north,-G(1,3));

% Differential GPS
Rd = Rm-Gm;

% Plot
figure(1);
plot(Gm(:,1),Gm(:,2),'k',Rm(:,1),Rm(:,2),'b',Rd(:,1),Rd(:,2),'r');
xlabel('magnetic north-south [m]');
ylabel('magnetic east-west [m]');
legend('mean ground','mean robot','differential robot');

grid on;
