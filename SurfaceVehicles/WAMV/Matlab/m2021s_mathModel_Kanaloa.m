%{
    Kanaloa
    Math Model
    Created by Thomas West
    Editted by Mayah Walker and Alex Sidelev
    ENGR-496
    20210501
%}

clc;
close all;
clear all;


%% WAM-V Configuration

% Distance from centerline to thruster
yp = 1;
xp = 1;

% Propeller thrusts [N]; half the thruster outpuy
Ts = 0.8*250;
Tp = 1*250;
BTs = 100;
BTp = 100;
% Thruster Configuration
B1 = [1 1 0 0; 0 0 1 1; yp -yp xp -xp];
% Thruster input vector. INCLUDES HOLONOMIC THRUSTERS
u = [BTs BTp Ts Tp].';

% Propeller Force
tau = B1*u;
%physical properties of WAM-V
m = 180;            % mass [kg], scaled from VRX 
xg = 0.1;           % Center of gravity [m]
Iz = 446.0;

%% Hydrodynamic Coefficients Damping coefficient values estimated based on VRX values
Xudot = 0;              % kg
Yvdot = 0;              % kg
Yrdot = 0;             % kg-m
Nrdot = 0;
%most impact
Xu = 4*-51.3;
Yv = 6*-40.0;
%how much we are turning 
Nr = 22*-40.0;
Yr = 0.0;
Nv = 0.0;



%% Measured Data (VRX)
% import data from excel. VRX data in in ENU

sheetName = 'me482-2020f-VRXData-Kanaloa.xlsx';
sheetNum = 5;

% GPS ENU Velocities
% import as NED shown below. VRX is 100% in ENU 
% ENU to NED => [x,y,z] => [y, x, -z] 
VyDataXl = xlsread(sheetName, sheetNum, 'E3:E200');
VxDataXl = xlsread(sheetName, sheetNum, 'F3:F200');
VzDataXl = xlsread(sheetName, sheetNum, 'G3:G200').*(-1);

% IMU 
% Imported angular velocity in NED
VyRotDataXl = xlsread(sheetName, sheetNum, 'S3:S200');
VxRotDataXl = xlsread(sheetName, sheetNum, 'R3:R200');
VzRotDataXl = xlsread(sheetName, sheetNum, 'T3:T200').*(-1);

% Import thruster ratio
RatioDataS = xlsread(sheetName, sheetNum, 'I3:K200');
RatioDataP = xlsread(sheetName, sheetNum, 'J3:L200');
thrusterRatios = [RatioDataS RatioDataP]; %[Ts Tp]

% IMU Quaternions
% (e1,e2,e3,eta) = field.orientation.(x,y,z,w)
% Import orientation in quaternions
e1 = xlsread(sheetName, sheetNum, 'N3:N200');
e2 = xlsread(sheetName, sheetNum, 'O3:O200');
e3 = xlsread(sheetName, sheetNum, 'P3:P200');
eta = xlsread(sheetName, sheetNum, 'Q3:Q200');

% Import time vector
timeDataxl = xlsread(sheetName, sheetNum, 'A3:A200');
timeData = timeDataxl;
% Convert to seconds
for i = 1:length(timeData)
    timeData(i) = (timeData(i)-timeDataxl(1))*10.^-9;
end

% Convert from Quaternions to Euler angles
quat = [eta e1 e2 e3];
seq = "XYZ";
eulXYZ = quat2eul(quat, seq);


phi = eulXYZ(:,1);
theta = eulXYZ(:,2);

%PSI should be altered by +pi/2
psi = eulXYZ(:,3)+pi/2;

zerovec = zeros(length(psi), 1);
onevec = ones(length(psi), 1);

% Rotation matrix for linear velocity from NED to Body
% Simplified for 3 DOF to rotation about Z
% Convert linear velocities from NED to Body
% x := surge
% y := sway
% psi := yaw angle
% Vel := velocity
% b := body frame
% n := NED frame
% VRX := measured data
xnVelVRX = VxDataXl;
ynVelVRX = VyDataXl;
psinVRX = VzRotDataXl;
xbVelVRX = onevec;
ybVelVRX = onevec;
psibVRX = psinVRX;

for i = 1:length(onevec)
    xbVelVRX(i) = xnVelVRX(i)*cos(psi(i)) + ynVelVRX(i)*sin(psi(i));
    ybVelVRX(i) = -xnVelVRX(i)*sin(psi(i)) + ynVelVRX(i)*cos(psi(i));
end

% VRX body position
ybVRX = cumtrapz(timeData,ybVelVRX);
xbVRX = cumtrapz(timeData,xbVelVRX);

% Rotation matrix for angular velocity from NED to Body...(+ x&y ,xt pi/2,
% anything else?) tait bryant angles 
% 3d matrix [data index, rotation mat, rotation mat]

%TINV IS CAUSING PROBLEMS?
Tinv = [onevec zerovec -sin(theta)]; 
Tinv(:,:,2)=[zerovec cos(phi) cos(theta).*sin(phi)];
Tinv(:,:,3) = [zerovec -sin(phi) cos(theta).*cos(phi)];

% Convert angular velocities from NED to Body
NEDAngVel = [VxRotDataXl VyRotDataXl VzRotDataXl].';
angVelData = zeros(length(VxRotDataXl), 3);
for i=1:size(VxRotDataXl)
    angVelData(i,:) = reshape(Tinv(i,:,:),3,3)*NEDAngVel(:,i);
end

VpData = angVelData(:,1);
VqData = angVelData(:,2);
VrData = angVelData(:,3);

% Measured NED Position 
ynVRX = cumtrapz(timeData,VyDataXl);
xnVRX = cumtrapz(timeData,VxDataXl);


%% Fossen's Linearized Dynamic Positioning (DP) Model 
syms x(t) y(t) yaw(t)
Eq1 =  (m - Xudot)*diff(x,2) - Xu*diff(x) == tau(1);
Eq2 = (m - Yvdot)*diff(y,2) + (m*xg - Yrdot)*diff(yaw,2) - Yv*diff(y) - Yr*diff(yaw) == tau(2);
Eq3 = (m*xg - Yrdot)*diff(y,2) + (Iz - Nrdot)*diff(yaw,2) - Nv*diff(y) - Nr*diff(yaw) == tau(3);
[ODE,Vars] = odeToVectorField([Eq1 Eq2 Eq3]);

% Print Vars then copy and paste below in comment block
%{
    y
    Dy
    x
    Dx
    yaw
    Dyaw
%}

F = matlabFunction(ODE, 'Vars',{'t','Y'});

% ic := initial conditions
ic = [ybVRX(1) ybVelVRX(1) xbVRX(1) xbVelVRX(1) psi(1) VrData(1)];
%should be surge, sway (y,t) aka (v,u)
[t, y] = ode45(@(t,Y)F(t,Y),timeData,ic);


%% Math Model Solutions
% x := surge
% y := sway
% psi := yaw angle
% Vel := velocity
% b := body frame
% n := NED frame
% MM := Math Model

% Body solutions (I made these positive, should I remove a pi/2?)
xbMM = y(:,3);
ybMM = y(:,1);
psibMM = y(:,5) + pi/2;
xbVelMM = y(:,4);
ybVelMM = y(:,2);
psibVelMM = y(:,6) + pi;

% NED solutions
xnMM = onevec;
ynMM = onevec;
psinMM = psibMM;
xnVelMM = onevec;
ynVelMM = onevec;
psinVelMM = psibVelMM;
%X SHOULD BE POSITIVE AND Y SHOULD BE NEGATIVE ENU AND NED ARE BOTH +
for i = 1:length(onevec)
    xnMM(i) = xbMM(i)*cos(psibMM(i)) + ybMM(i)*sin(psibMM(i));
    ynMM(i) = xbMM(i)*sin(psibMM(i)) - ybMM(i)*cos(psibMM(i));
    xnVelMM(i) = xbVelMM(i)*cos(psibVelMM(i)) + ybVelMM(i)*sin(psibVelMM(i));
    ynVelMM(i) = xbVelMM(i)*sin(psibVelMM(i)) - ybVelMM(i)*cos(psibVelMM(i));
end


% WE SHOULD MAKE EVERYTHING IN ENU..CURRENTLY ROTATION MATRIX IS IN NED ,
% BUT VRX IS ENU.

%% Plotting in NED (HELP)

figure('Name','NED','WindowState', 'maximized')
set(gca,'FontSize',20)
subplot(1,2,1)
% changed 
plot(ynVRX, xnVRX, 'b')
hold on
plot(ynMM, xnMM, 'r')
axis equal
xlabel('East [m]')
ylabel('North [m]')
legend('VRX','Math Model')
title('NED Position')
set(gca,'FontSize',20)
%figure('Name','NED','WindowState', 'maximized')
set(gca,'FontSize',20)
subplot(1,2,2)
plot(VyDataXl, VxDataXl, 'b')
hold on
plot(ynVelMM, xnVelMM, 'r')
axis equal
xlabel('East [m/s]')
ylabel('North [m/s]')
legend('VRX','Math Model')
title('NED Velocity')
set(gca,'FontSize',20)

