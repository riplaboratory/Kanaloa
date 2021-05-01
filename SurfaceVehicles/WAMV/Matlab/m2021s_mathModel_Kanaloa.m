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
B1 = [1 1 0 0 ; 1 1 0 0; - yp yp xp -xp];
% Thruster input vector. INCLUDE HOLONOMIC THRUSTERS?
u = [BTs BTp Ts Tp].';

% Propeller Force
tau = B1*u;
%physical properties of WAM-V
m = 180;            % mass [kg], scaled from VRX 
xg = 0.1;           % Center of gravity [m]
Iz = 446.0

%% Hydrodynamic Coefficients
% 

% Initial coefficients from Osaka and VRX
% Added Masses
% Xudot = -0.84;              % g
% Yvdot = -1.12;              % kg
% Yrdot = -0.435;             % kg-m
% Nrdot = -0.089;             % kg-m^2
% Damping coefficients
% Xu = -45 * 4.75;            % kg/s
% Yv = -40 * 5;               % kg/s, change from -1.155 to -40 from VRX and scaled by 5
% Yr = -0.5*550;              % kg-m/s
% Nv = -0.265*550;            % kg-m/s
% Nr = -400 *1;               % kg-m^2/s, changed from -0.177 to -400 from VRX

% Damping coefficients for Small turning
% Xu = -45 * 4.75;          % kg/s, made negative and scaled by 5
% Yv = -40 * 10;            % kg/s, change from -1.155 to -40 from VRX
% Yr = -0.5*500;            % kg-m/s
% Nv = -0.265*500;          % kg-m/s
% Nr = -400*5;              % kg-m^2/s, changed from -0.177 to -400 from VRX *5

% Estimated coefficients using Parameter Estimation in Simulink:
% Added Masses
% Nrdot = -0.63632;
% Xudot = -0.042239;
% Yrdot = -0.44716;
% Yvdot = -1.1276;
% % Damping coefficients
% Xu = -435.39;
% Yr = -0.49303;
% Yv = -1.0151;
% Nr = -2.3282;
% Nv = -1.2195;

% % Second Estimated Parameters
% Nr = -3293.8;
% Nv = -1628.4;
% Xu = -216.03;
% Yr = 2318.3;
% Yv = 1051.3;
% 
% % Third Estimated Parameters
% Nr = -8085.9;
% Nv = -121.33;
% Xu = -215.88;
% Yr = -31.854;
% Yv = -8743.9;

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
% import as NED shown below
VyDataXl = xlsread(sheetName, sheetNum, 'E3:E200').*(-1);
VxDataXl = xlsread(sheetName, sheetNum, 'F3:F200').*(-1);
VzDataXl = xlsread(sheetName, sheetNum, 'G3:G200').*(-1);

% IMU 
% Imported angular velocity in NED
VyRotDataXl = xlsread(sheetName, sheetNum, 'R3:R200').*(-1);
VxRotDataXl = xlsread(sheetName, sheetNum, 'S3:S200').*(-1);
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

%SHOULD PSI BE -PI/2 OR POSITIVE?
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

%% Animated Plotting

% figure('Name','Animated Plot in NED','WindowState', 'maximized')
% set(gca,'FontSize',20);
% for i = 1:length(VxDataXl)-1
%     subplot(1,2,1)
%     plot([ynVRX(i) ynVRX(i+1)], [xnVRX(i) xnVRX(i+1)], 'b')
%     hold on
%     plot([ynMM(i) ynMM(i+1)], [xnMM(i) xnMM(i+1)], 'r')
%     axis equal
%     xlabel('East')
%     ylabel('North')
%     legend('VRX','Math Model')
%     title('NED Position')
% 
%     subplot(1,2,2)
%     plot([VyDataXl(i) VyDataXl(i+1)], [VxDataXl(i) VxDataXl(i+1)], 'b')
%     hold on
%     plot([ynVelMM(i) ynVelMM(i+1)], [xnVelMM(i) xnVelMM(i+1)], 'r')
%     axis equal
%     xlabel('East')
%     ylabel('North')
%     legend('VRX','Math Model')
%     title('NED Velocity')
%     
%     pause(0.1)
% end



%% Prewritten Code to Import Data

% % % % % %{
% % 
% % Copy and Paste code to be read from Excel.
% % 
% % Adjust Thurster Configuration.
% % 
% % 
% % (1) Full Surge
% % 
% % %Propeller thrusts
% % Tp = 1*250;
% % Ts = 1*250;
% % 
% % sheetName = 'me482-2020f-VRXData-Kanaloa.xlsx';
% % sheetNum = 1;
% % 
% % % GPS ENU Velocities
% % % import as NED shown below
% % VyDataXl = xlsread(sheetName, sheetNum, 'E3:E157');
% % VxDataXl = xlsread(sheetName, sheetNum, 'F3:F157')*-1;
% % VzDataXl = xlsread(sheetName, sheetNum, 'G3:G157')*(-1);
% % 
% % % IMU 
% % % Imported angular velocity in NED
% % VyRotDataXl = xlsread(sheetName, sheetNum, 'U3:U157');
% % VxRotDataXl = xlsread(sheetName, sheetNum, 'V3:V157');
% % VzRotDataXl = xlsread(sheetName, sheetNum, 'W3:W157').*(-1);
% % 
% % % Import thruster ratio
% % RatioDataS = xlsread(sheetName, sheetNum, 'K3:K157');
% % RatioDataP = xlsread(sheetName, sheetNum, 'L3:L157');
% % thrusterRatios = [RatioDataS RatioDataP]; %[Ts Tp]
% % 
% % % IMU Quaternions
% % % (e1,e2,e3,eta) = field.orientation.(x,y,z,w)
% % % Import orientation in quaternions
% % e1 = xlsread(sheetName, sheetNum, 'Q3:Q157');
% % e2 = xlsread(sheetName, sheetNum, 'R3:R157');
% % e3 = xlsread(sheetName, sheetNum, 'S3:S157');
% % eta = xlsread(sheetName, sheetNum, 'T3:T157');
% % 
% % % Import time vector
% % timeDataxl = xlsread(sheetName, sheetNum, 'A3:A157');
% % 
% % 
% % 
% % (2) Small Turn
% % 
% % %Propeller thrusts
% % Ts = 1*250;
% % Tp = 0.9*250;
% % 
% % sheetName = 'me482-2020f-VRXData-Kanaloa.xlsx';
% % sheetNum = 2;
% % 
% % % GPS ENU Velocities
% % % import as NED shown below
% % VyDataXl = xlsread(sheetName, sheetNum, 'E3:E127');
% % VxDataXl = xlsread(sheetName, sheetNum, 'F3:F127')*-1;
% % VzDataXl = xlsread(sheetName, sheetNum, 'G3:G127')*(-1);
% % 
% % % IMU 
% % % Imported angular velocity in NED
% % VyRotDataXl = xlsread(sheetName, sheetNum, 'R3:R127');
% % VxRotDataXl = xlsread(sheetName, sheetNum, 'S3:S127');
% % VzRotDataXl = xlsread(sheetName, sheetNum, 'T3:T127').*(-1);
% % 
% % % Import thruster ratio
% % RatioDataS = xlsread(sheetName, sheetNum, 'I3:K127');
% % RatioDataP = xlsread(sheetName, sheetNum, 'J3:L127');
% % thrusterRatios = [RatioDataS RatioDataP]; %[Ts Tp]
% % 
% % % IMU Quaternions
% % % (e1,e2,e3,eta) = field.orientation.(x,y,z,w)
% % % Import orientation in quaternions
% % e1 = xlsread(sheetName, sheetNum, 'N3:N127');
% % e2 = xlsread(sheetName, sheetNum, 'O3:O127');
% % e3 = xlsread(sheetName, sheetNum, 'P3:P127');
% % eta = xlsread(sheetName, sheetNum, 'Q3:Q127');
% % 
% % % Import time vector
% % timeDataxl = xlsread(sheetName, sheetNum, 'A3:A127');
% % 
% % 
% % 
% % (3) Circle
% % 
% % %Propeller thrusts
% % Ts = 0.5*250;
% % Tp = 1*250;
% % 
% % sheetName = 'me482-2020f-VRXData-Kanaloa.xlsx';
% % sheetNum = 3;
% % 
% % % GPS ENU Velocities
% % % import as NED shown below
% % VyDataXl = xlsread(sheetName, sheetNum, 'B2:B379');
% % VxDataXl = xlsread(sheetName, sheetNum, 'C2:C379')*-1;
% % VzDataXl = xlsread(sheetName, sheetNum, 'D2:D379')*(-1);
% % 
% % % IMU 
% % % Imported angular velocity in NED
% % VyRotDataXl = xlsread(sheetName, sheetNum, 'I2:I379');
% % VxRotDataXl = xlsread(sheetName, sheetNum, 'J2:J379');
% % VzRotDataXl = xlsread(sheetName, sheetNum, 'K2:K379').*(-1);
% % 
% % % Import thruster ratio
% % RatioDataS = xlsread(sheetName, sheetNum, 'R2:R379');
% % RatioDataP = xlsread(sheetName, sheetNum, 'S2:S379');
% % thrusterRatios = [RatioDataS RatioDataP]; %[Ts Tp]
% % 
% % % IMU Quaternions
% % % (e1,e2,e3,eta) = field.orientation.(x,y,z,w)
% % % Import orientation in quaternions
% % e1 = xlsread(sheetName, sheetNum, 'E2:E379');
% % e2 = xlsread(sheetName, sheetNum, 'F2:F379');
% % e3 = xlsread(sheetName, sheetNum, 'G2:G379');
% % eta = xlsread(sheetName, sheetNum, 'H2:H379');
% % 
% % % Import time vector
% % timeDataxl = xlsread(sheetName, sheetNum, 'A2:A379');
% % 
% % 
% % 
% % (4) Small Turn
% % 
% % %Propeller thrusts
% % Ts = 1*250;
% % Tp = 0.8*250;
% % 
% % sheetName = 'me482-2020f-VRXData-Kanaloa.xlsx';
% % sheetNum = 4;
% % 
% % % GPS ENU Velocities
% % % import as NED shown below
% % VyDataXl = xlsread(sheetName, sheetNum, 'E3:E47');
% % VxDataXl = xlsread(sheetName, sheetNum, 'F3:F47')*-1;
% % VzDataXl = xlsread(sheetName, sheetNum, 'G3:G47')*(-1);
% % 
% % % IMU 
% % % Imported angular velocity in NED
% % VyRotDataXl = xlsread(sheetName, sheetNum, 'R3:R47');
% % VxRotDataXl = xlsread(sheetName, sheetNum, 'S3:S47');
% % VzRotDataXl = xlsread(sheetName, sheetNum, 'T3:T47').*(-1);
% % 
% % % Import thruster ratio
% % RatioDataS = xlsread(sheetName, sheetNum, 'I3:K47');
% % RatioDataP = xlsread(sheetName, sheetNum, 'J3:L47');
% % thrusterRatios = [RatioDataS RatioDataP]; %[Ts Tp]
% % 
% % % IMU Quaternions
% % % (e1,e2,e3,eta) = field.orientation.(x,y,z,w)
% % % Import orientation in quaternions
% % e1 = xlsread(sheetName, sheetNum, 'N3:N47');
% % e2 = xlsread(sheetName, sheetNum, 'O3:O47');
% % e3 = xlsread(sheetName, sheetNum, 'P3:P47');
% % eta = xlsread(sheetName, sheetNum, 'Q3:Q47');
% % 
% % % Import time vector
% % timeDataxl = xlsread(sheetName, sheetNum, 'A3:A47');
% % 
% % 
% % 
% % (5) New Circle (Full)
% % 
% % %Propeller thrusts
% % Ts = 0.5*250;
% % Tp = 1*250;
% % 
% % sheetName = 'me482-2020f-VRXData-Kanaloa.xlsx';
% % sheetNum = 5;
% % 
% % % GPS ENU Velocities
% % % import as NED shown below
% % VyDataXl = xlsread(sheetName, sheetNum, 'E3:E895');
% % VxDataXl = xlsread(sheetName, sheetNum, 'F3:F895')*-1;
% % VzDataXl = xlsread(sheetName, sheetNum, 'G3:G895')*(-1);
% % 
% % % IMU 
% % % Imported angular velocity in NED
% % VyRotDataXl = xlsread(sheetName, sheetNum, 'R3:R895');
% % VxRotDataXl = xlsread(sheetName, sheetNum, 'S3:S895');
% % VzRotDataXl = xlsread(sheetName, sheetNum, 'T3:T895').*(-1);
% % 
% % % Import thruster ratio
% % RatioDataS = xlsread(sheetName, sheetNum, 'I3:K895');
% % RatioDataP = xlsread(sheetName, sheetNum, 'J3:L895');
% % thrusterRatios = [RatioDataS RatioDataP]; %[Ts Tp]
% % 
% % % IMU Quaternions
% % % (e1,e2,e3,eta) = field.orientation.(x,y,z,w)
% % % Import orientation in quaternions
% % e1 = xlsread(sheetName, sheetNum, 'N3:N895');
% % e2 = xlsread(sheetName, sheetNum, 'O3:O895');
% % e3 = xlsread(sheetName, sheetNum, 'P3:P895');
% % eta = xlsread(sheetName, sheetNum, 'Q3:Q895');
% % 
% % % Import time vector
% % timeDataxl = xlsread(sheetName, sheetNum, 'A3:A895');
% % 
% % 
% % 
% % (6) New Circle (Partial)
% % 
% % %Propeller thrusts
% % Ts = 0.5*250;
% % Tp = 1*250;
% % 
% % sheetName = 'me482-2020f-VRXData-Kanaloa.xlsx';
% % sheetNum = 5;
% % 
% % % GPS ENU Velocities
% % % import as NED shown below
% % VyDataXl = xlsread(sheetName, sheetNum, 'E3:E240');
% % VxDataXl = xlsread(sheetName, sheetNum, 'F3:F240')*-1;
% % VzDataXl = xlsread(sheetName, sheetNum, 'G3:G240')*(-1);
% % 
% % % IMU 
% % % Imported angular velocity in NED
% % VyRotDataXl = xlsread(sheetName, sheetNum, 'R3:R240');
% % VxRotDataXl = xlsread(sheetName, sheetNum, 'S3:S240');
% % VzRotDataXl = xlsread(sheetName, sheetNum, 'T3:T240').*(-1);
% % 
% % % Import thruster ratio
% % RatioDataS = xlsread(sheetName, sheetNum, 'I3:K240');
% % RatioDataP = xlsread(sheetName, sheetNum, 'J3:L240');
% % thrusterRatios = [RatioDataS RatioDataP]; %[Ts Tp]
% % 
% % % IMU Quaternions
% % % (e1,e2,e3,eta) = field.orientation.(x,y,z,w)
% % % Import orientation in quaternions
% % e1 = xlsread(sheetName, sheetNum, 'N3:N240');
% % e2 = xlsread(sheetName, sheetNum, 'O3:O240');
% % e3 = xlsread(sheetName, sheetNum, 'P3:P240');
% % eta = xlsread(sheetName, sheetNum, 'Q3:Q240');
% % 
% % % Import time vector
% % timeDataxl = xlsread(sheetName, sheetNum, 'A3:A240');
% % 
% % %}