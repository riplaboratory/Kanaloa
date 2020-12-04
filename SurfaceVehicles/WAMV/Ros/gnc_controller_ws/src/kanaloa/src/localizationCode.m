% THRUSTER CONFIG FILE
% Distance from centerline to thruster
yp = 1.22;
% Max Propeller thrusts
Ts = 250;
Tp = 250;
maxThrust = [Ts Tp];
% Thruster Configuration
B1 = [1 1; 0 0; -yp yp];
% Import thruster ratio
RatioDataP = xlsread('Simulation1.xlsx', 1, 'S2:S379');
RatioDataS = xlsread('Simulation1.xlsx', 1, 'R2:R379');
thrusterRatios = [RatioDataS RatioDataP].'; %[Ts Tp]
% Thruster forces: [Fx, Fy, Mr]
tau = B1.*maxThrust*thrusterRatios;


% MASS/INERTIA FILE
% Total mass of boat
m = 510;
% x direction offset from origin
xg = 0;
% Inertia about z
Iz = 459.748;

% HYDRODYNAMIC DAMPING/ADDED MASS COEFFICIENTS FILE
% Estimated parameters:
Nr = 2.3282;
Nrdot = 0.63632;
Nv = 1.2195;
Xu = -435.39;
Xudot = 0.042239;
Yr = 0.49303;
Yrdot = 0.44716;
Yv = 1.0151;
Yvdot = 1.1276;


% Mass and damping matrices for context
% M = [(m-Xudot) 0 0; 0 (m-Yvdot) (m*xg-Yrdot); 0 (m*xg-Yrdot) (Iz-Nrdot)];
% D1 = [-Xu 0 0; 0 -Yv -Yr; 0 -Nv -Nr];

% FINDING EULER ANGLE PSI FROM IMU DATA
% Possibly a separate file for this?
% Need psi for Body-NED conversions
% Import orientation in quaternions
e1 = xlsread('Simulation1.xlsx', 1, 'E2:E379');
e2 = xlsread('Simulation1.xlsx', 1, 'F2:F379');
e3 = xlsread('Simulation1.xlsx', 1, 'G2:G379');
eta = xlsread('Simulation1.xlsx', 1, 'H2:H379');
% Convert from Quaternions to Euler angles
R11 = (1 - (e2.^2 + e3.^2).*2);
R21 = (e1.*e2 + e3.*eta).*2;
R32 = (e2.*e3 + e1.*eta).*2;
R33 = 1 - (e1.^2 + e2.^2).*2;
R31 = (e1.*e3 - e2.*eta).*2;
% Euler angles
theta = -asind(R31);
phi = atan2(R32, R33);
psi = atan2(R21, R11);


% MATH MODEL FILE
% Take input forces (tau), output velocities in NED
% System of equations, Eq1 = x, Eq2 = y, Eq3 = r
syms x(t) y(t) r(t)
Eq1 =  -Xu*diff(x) - diff(x,2)*(Xudot - m) == tau(1);
Eq2 = - Yr*diff(r) - Yv*diff(y) - diff(y,2)*(Yvdot - m) - diff(r,2)*(Yrdot - m*xg) == tau(2);
Eq3 = diff(r,2)*(Iz - Nrdot) - Nv*diff(y) - Nr*diff(r) - diff(y,2)*(Yrdot - m*xg) == tau(3);
% Turning system of equations into a function to be solved
[ODE,Vars] = odeToVectorField([Eq1 Eq2 Eq3]);
F = matlabFunction(ODE, 'Vars',{'t','Y'});
% initial conditions = [r, Vr, x, Vx, y, Vy]
% Make initial conditions = previous velocity vector from EKF
ic = [0, 0, 0, 0, 0, 0];
% Matlab ODE solver
[t, y] = ode45(@(t,Y)F(t,Y),timeData,ic);
% Will create a vector y = [r Vr x Vx y Vy]
%
% Transform velocities from Body to ENU:
% Rotation matrix for linear velocity from Body to NED
% Simplified for 3 DOF to rotation about Z
% 3d matrix [data index, rotation mat, rotation mat]
Rz = [cos(psi) -sin(psi) zerovec]; 
Rz(:,:,2)=[sin(psi), cos(psi), zerovec];
Rz(:,:,3) = [zerovec, zerovec, onevec];
% Velocities in Body [Vx, Vy, Vr]
bodyVelocities = [y(:,4) y(:,6) y(:,2)].';
% Convert linear velocities from Body to NED
NEDVelocities = zeros(length(psi), 3).';
for i=1:size(psi)
    NEDVelocities(:,i) = reshape(Rz(i,:,:),3,3)*bodyVelocities(:,i);
end
% Transform velocities from NED to ENU
% ENUVelocities = [Vx, Vy, Vr]
ENUVelocities = zeros(size(NEDVelocities));
ENUVelocities(1,:) = NEDVelocities(2,:);
ENUVelocities(2,:) = NEDVelocities(1,:);
ENUVelocities(3,:) = -1*NEDVelocities(3,:);
% Use ENUVelocities for EKF comparison



% My plot for sanity check
figure(1)
plot(NEDVelocities(2,:),NEDVelocities(1,:),'r')
title('North vs East Velocity, Comparison')
xlabel('Vy (East)')
ylabel('Vx (North)')


