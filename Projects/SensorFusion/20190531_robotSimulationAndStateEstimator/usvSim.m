classdef usvSim
        
    methods (Static)
        
        function [t,lp,ic] = simDefaults()
            % Sets simulation variables to their defaults.  Can change the
            % values in this function OR edit the structure yourself.
            
            % USER INPUT: time properties
            t.dt = 0.001;   % simulation time step [s]
            t.start = 0;    % simulation start time [s]
            t.end = 30;     % simulation end time [s]

            % USER INPUT: USV lumped parameters
            lp.m = 225;     % mass/inertia [kg]
            lp.I = 100;     % rotational moment of inertia [kg*m^2]
            lp.bxr = 40;    % drag in the surge (robot frame along x) direction [N*s/m]
            lp.byr = 400;   % drag in the sway (robot frame along y) direction [N*s/m]
            lp.btzr = 300;  % rotational drag in the yaw (robot frame about z) direction [N*s/m]

            % USER INPUT: initial conditions in the map frame
            ic.x0 = 0;      % initial position (robot frame along x) [m] 
            ic.y0 = 0;      % initial position (robot frame along y) [m]
            ic.tz0 = 0;     % initial angle (robot frame about z) [rad]
            ic.dx0 = 0;     % initial velocity (robot frame along x) [m/s] 
            ic.dy0 = 0;     % initial velocity (robot frame along y) [m/s] 
            ic.dtz0 = 0;    % initial angular velocity (robot about z) [rad/s] 
            
            % Dependent variables
            t.N = round(t.end/t.dt - t.start/t.dt);   % total number of time steps [ ]
            t.now = t.start;
        
        end
        
        function [sim,sen] = simOutputSetup(t,sen)
            
            % USER INPUT: sensor setup
            sen.gps.rr = 1;     % refresh rate of GPS sensor [Hz]
            sen.gps.snr = 0.1;  % signal-to-noise ratio of GPS sensor [ ]
            sen.imu.rr = 100;   % refresh rate of IMU sensor [Hz]
            sen.imu.snr = 40;   % signal-to-noise ratio of IMU sensor [ ]
            
            % State, output, and input vectors
            sim.xr = zeros(9,t.N);  % state vector (robot reference frame)
            sim.yr = zeros(9,t.N);  % output vector (robot reference frame)
            sim.xm = zeros(9,t.N);  % state vector (map reference frame)
            sim.ym = zeros(9,t.N);  % output vector (map reference frame)
            sim.u = zeros(3,t.N);   % input vector (control inputs act on robot frame)
            
            % Sensor data vectors
            sen.gps.xr = zeros(9,t.end/(1/sen.gps.rr));   % gps data vector (robot reference frame)
            sen.gps.xm = zeros(9,t.end/(1/sen.gps.rr));   % gps data vector (map reference frame)
            sen.imu.xr = zeros(9,t.end/(1/sen.imu.rr));   % imu data vector (robot reference frame)
            sen.imu.xm = zeros(9,t.end/(1/sen.imu.rr));   % imu data vector (map reference frame)
            
            % Sensor update tracker
            sen.gps.update = 0;         % gps update tracker (boolean)
            sen.imu.update = 0;         % imu update tracker (boolean)
            sen.gps.k = 1;              % gps iteration tracker
            sen.imu.k = 1;              % imu iteration tracker
            sen.gps.lastUpdateTime = 0; % gps last update time tracker [s]
            sen.imu.lastUpdateTime = 0; % imu last update time tracker [s]
            
        end
        
        function p = createPlant(lp)
            % Creates system, intput, output and feedthrough matrices for
            % system representation.  Note that these matrices are in the
            % robot reference frame.     
            
            % State matrix
            p.A = [...
                0 0 0 1 0 0 0 0 0;...
                0 0 0 0 1 0 0 0 0;...
                0 0 0 0 0 1 0 0 0;...
                0 0 0 -lp.bxr/lp.m 0 0 0 0 0;...
                0 0 0 0 -lp.byr/lp.m 0 0 0 0;...
                0 0 0 0 0 -lp.btzr/lp.I 0 0 0;...
                0 0 0 0 0 0 0 0 0;...
                0 0 0 0 0 0 0 0 0;...
                0 0 0 0 0 0 0 0 0];
            
            % Input matrix
            p.B = [...
                0 0 0;...
                0 0 0;...
                0 0 0;...
                1/lp.m 0 0;...
                0 1/lp.m 0;...
                0 0 1/lp.I;...
                0 0 0;...
                0 0 0;...
                0 0 0];
            
            % Output matrix
            p.C = [...
                eye(6,9);...
                p.A(4:6,:)];
            
            % Feedthrough matrix
            p.D = [...
                zeros(6,3);...
                p.B(4:6,:)];
                                    
        end
        
        function sim = insertIC(sim,ic,obj)
            
            % Insert initial conditions
            sim.xm(1,1) = ic.x0;
            sim.xm(2,1) = ic.y0;
            sim.xm(3,1) = ic.tz0;
            sim.xm(4,1) = ic.dx0;
            sim.xm(5,1) = ic.dy0;
            sim.xm(6,1) = ic.dtz0;
            sim.ym(:,1) = sim.xm(:,1);
            sim.xr(:,1) = obj.tf_m2r(sim.xm(:,1),ic.tz0,ic.dtz0,0);
            sim.yr(:,1) = obj.tf_m2r(sim.xm(:,1),ic.tz0,ic.dtz0,0);
            
        end
        
        function caEst = caEstDefaults()
            % Constant acceleration state-transition model default values.
            
            % USER INPUT: time properties
            caEst.dt = 0.005;   % estimator time step [s]
            
            % USER INPUT: process noise covariance
            caEst.Q = [...
                10 0 0 0 0 0 0 0 0;...       % [m]
                0 10 0 0 0 0 0 0 0;...       % [m]
                0 0 10 0 0 0 0 0 0;...       % [rad]
                0 0 0 1E6 0 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 1E6 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 0 1 0 0 0;...     % [rad/s]
                0 0 0 0 0 0 1E6 0 0;...     % [m/s^2]
                0 0 0 0 0 0 0 1E6 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0 1E6];         % [rad/s^2] (not measured)
    
            % USER INUT: measurement noise covariance       
            caEst.R = [...
                0.5 0 0 0 0 0 0 0 0;...     % [m]
                0 0.5 0 0 0 0 0 0 0;...     % [m]
                0 0 0.1 0 0 0 0 0 0;...     % [rad]
                0 0 0 1 0 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 1 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 0 0.5 0 0 0;...     % [rad/s]
                0 0 0 0 0 0 0.2 0 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0.2 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0 1];         % [rad/s] (not measured)

            % USER INPUT: observation matrix 
            caEst.H = [...
                1 0 0 0 0 0 0 0 0;...       % measured
                0 1 0 0 0 0 0 0 0;...       % measured
                0 0 1 0 0 0 0 0 0;...       % measured
                0 0 0 0 0 0 0 0 0;...       % not measured
                0 0 0 0 0 0 0 0 0;...       % not measured
                0 0 0 0 0 1 0 0 0;...       % measured
                0 0 0 0 0 0 1 0 0;...       % measured
                0 0 0 0 0 0 0 1 0;...       % measured
                0 0 0 0 0 0 0 0 0];         % not measured
                    
        end
        
        function caest = caEstOutputSetup(t,caest)
           
            % Instantiate estimator matrices            
            caest.xm = zeros(9,t.end/caest.dt);     % state estimate in map frame
            caest.xr = zeros(9,t.end/caest.dt);     % state estimate in robot frame
            caest.Lx = zeros(9,9,t.end/caest.dt);   % kalman gain matrix
            caest.Px = zeros(9,9,t.end/caest.dt);   % covariance matrix            
            caest.k = 1;                            % iteration tracking variable
            
        end
        
        function mcaEst = mcaEstDefaults()
            % Discrete derivative state-transition model default values.
            
            % USER INPUT: time properties
            mcaEst.dt = 0.005;   % estimator time step [s]

            % USER INPUT: process noise covariance
            mcaEst.Q = [...
                1 0 0 0 0 0 0 0 0;...    % [m]
                0 1 0 0 0 0 0 0 0;...    % [m]
                0 0 0.01 0 0 0 0 0 0;...    % [rad]
                0 0 0 1 0 0 0 0 0;...       % [m/s]
                0 0 0 0 1 0 0 0 0;...       % [m/s]
                0 0 0 0 0 0.01 0 0 0;...    % [rad/s]
                0 0 0 0 0 0 1 0 0;...       % [m/s^2]
                0 0 0 0 0 0 0 1 0;...       % [m/s^2]
                0 0 0 0 0 0 0 0 1];         % [rad/s^2] (not measured)

            % USER INUT: measurement noise covariance       
            mcaEst.R = [...
                0.5 0 0 0 0 0 0 0 0;...     % [m]
                0 0.5 0 0 0 0 0 0 0;...     % [m]
                0 0 0.1 0 0 0 0 0 0;...     % [rad]
                0 0 0 1 0 0 0 0 0;...       % [m/s]
                0 0 0 0 1 0 0 0 0;...       % [m/s]
                0 0 0 0 0 0.5 0 0 0;...     % [rad/s]
                0 0 0 0 0 0 0.2 0 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0.2 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0 1];         % [rad/s] (not measured)

            % USER INPUT: observation matrix 
            mcaEst.H = [...
                1 0 0 0 0 0 0 0 0;...       % measured
                0 1 0 0 0 0 0 0 0;...       % measured
                0 0 1 0 0 0 0 0 0;...       % measured
                0 0 0 1 0 0 0 0 0;...       % pesudo measured
                0 0 0 0 1 0 0 0 0;...       % pseudo measured
                0 0 0 0 0 1 0 0 0;...       % measured
                0 0 0 0 0 0 1 0 0;...       % measured
                0 0 0 0 0 0 0 1 0;...       % measured
                0 0 0 0 0 0 0 0 0];         % not measured
            
            % USER INPUT: the number of past sensor updates used to
            % calculate the incremental moving mean for sensor data.
            mcaEst.gps.n = 6;   % must be 2 or more
            mcaEst.imu.n = 2;   % must be 2 (for now, mathematically averaging IMU measurements is unecessary)
            
        end
        
        function mcaEst = mcaEstOutputSetup(t,mcaEst)
            
            % Instantiate sensor matrices
            mcaEst.gpsArray = zeros(9,mcaEst.gps.n);        % past GPS measurements
            mcaEst.gpsLastUpdateTime = 0;                   % last GPS update time [s]
            mcaEst.imuArray = zeros(9,2);                   % past IMU measurements
            mcaEst.imuLastUpdateTime = 0;                   % last IMU update time [s]
            mcaEst.meas.all = zeros(9,t.end/mcaEst.dt);     % measurements from all sensors
            mcaEst.meas.gps = zeros(9,t.end/mcaEst.dt);     % measurements from GPS only
            mcaEst.meas.imu = zeros(9,t.end/mcaEst.dt);     % measurements from IMU only
            mcaEst.varGain = zeros(9,9,t.end/mcaEst.dt);    % variance gain matrix
            
            % Instantiate estimator matrices
            mcaEst.xm = zeros(9,t.end/mcaEst.dt);       % state estimate in map frame
            mcaEst.xr = zeros(9,t.end/mcaEst.dt);       % state estimate in robot frame
            mcaEst.Lx = zeros(9,9,t.end/mcaEst.dt);     % kalman gain matrix
            mcaEst.Px = zeros(9,9,t.end/mcaEst.dt);     % covariance matrix
            mcaEst.k = 1;                               % iteration tracking variable
                                    
        end
        
        function vecM = tf_r2m(vecR,tz,dtz,ddtz)
            
            vecM = [...
                                           cos(tz),                       -sin(tz), 0,              0,              0, 0,       0,        0, 0;...
                                           sin(tz),                        cos(tz), 0,              0,              0, 0,       0,        0, 0;...
                                                 0,                              0, 1,              0,              0, 0,       0,        0, 0;...
                                      -dtz*sin(tz),                   -dtz*cos(tz), 0,        cos(tz),       -sin(tz), 0,       0,        0, 0;...
                                       dtz*cos(tz),                   -dtz*sin(tz), 0,        sin(tz),        cos(tz), 0,       0,        0, 0;...
                                                 0,                              0, 0,              0,              0, 1,       0,        0, 0;...
                    - cos(tz)*dtz^2 - ddtz*sin(tz),   sin(tz)*dtz^2 - ddtz*cos(tz), 0, -2*dtz*sin(tz), -2*dtz*cos(tz), 0, cos(tz), -sin(tz), 0;...
                    - sin(tz)*dtz^2 + ddtz*cos(tz), - cos(tz)*dtz^2 - ddtz*sin(tz), 0,  2*dtz*cos(tz), -2*dtz*sin(tz), 0, sin(tz),  cos(tz), 0;...
                                                 0,                              0, 0,              0,              0, 0,       0,        0, 1]*vecR;


        end
        
        function vecR = tf_m2r(vecM,tz,dtz,ddtz)
        
            vecR = [...
                                           cos(tz),                        sin(tz), 0,              0,              0, 0,        0,       0, 0;...
                                          -sin(tz),                        cos(tz), 0,              0,              0, 0,        0,       0, 0;...
                                                 0,                              0, 1,              0,              0, 0,        0,       0, 0;...
                                      -dtz*sin(tz),                    dtz*cos(tz), 0,        cos(tz),        sin(tz), 0,        0,       0, 0;...
                                      -dtz*cos(tz),                   -dtz*sin(tz), 0,       -sin(tz),        cos(tz), 0,        0,       0, 0;...
                                                 0,                              0, 0,              0,              0, 1,        0,       0, 0;...
                    - cos(tz)*dtz^2 - ddtz*sin(tz), - sin(tz)*dtz^2 + ddtz*cos(tz), 0, -2*dtz*sin(tz),  2*dtz*cos(tz), 0,  cos(tz), sin(tz), 0;...
                      sin(tz)*dtz^2 - ddtz*cos(tz), - cos(tz)*dtz^2 - ddtz*sin(tz), 0, -2*dtz*cos(tz), -2*dtz*sin(tz), 0, -sin(tz), cos(tz), 0;...
                                                 0,                              0, 0,              0,              0, 0,        0,       0, 1]*vecM;
                                                  
        end
        
        function sim = controller(sim,k)

            if (k == 0)
                sim.u(1,k) = 0;     % force input in surge direction
                sim.u(2,k) = 0;     % force input in sway direction
                sim.u(3,k) = 0;     % torque input in yaw direction
            elseif (k > 1 && k < 10000)
                sim.u(1,k) = -150;  % force input in surge direction
                sim.u(2,k) = 0;     % force input in sway direction
                sim.u(3,k) = 100;   % torque input in yaw direction
            elseif (k >= 10000 && k < 20000)
                sim.u(1,k) = 0;     % force input in surge direction
                sim.u(2,k) = 0;     % force input in sway direction
                sim.u(3,k) = 0;     % torque input in yaw direction
            else
                sim.u(1,k) = 0;     % force input in surge direction
                sim.u(2,k) = 0;     % force input in sway direction
                sim.u(3,k) = 0;     % torque input in yaw direction
            end

        end
        
        function sim = runSim(t,p,sim,obj,k)

            F = eye(9)+p.A*t.dt;
            G = p.B*t.dt;                                           % discrete control input matrix
            H = p.C;                                                % discrete output matrix
            J = p.D;                                                % discrete feedthrough (direct transmission) matrix 
            if k ~= t.N

                % Simulate plant using discrete Euler integration
                sim.xr(:,k+1) = F*sim.xr(:,k)+...
                    G*[sim.u(1,k);sim.u(2,k);sim.u(3,k)];     % state matrix integration solution
                sim.yr(:,k+1) = H*sim.xr(:,k+1)+...
                    J*[sim.u(1,k);sim.u(2,k);sim.u(3,k)];     % state observer matrix integration solution
                                                
                % Convert robot frame to map frame
                sim.xm(:,k+1) = obj.tf_r2m(sim.xr(:,k+1),...
                    sim.xr(3,k+1),...
                    sim.xr(6,k+1),...
                    sim.xr(9,k+1));
                sim.ym(:,k+1) = obj.tf_r2m(sim.yr(:,k+1),...
                    sim.yr(3,k+1),...
                    sim.yr(6,k+1),...
                    sim.yr(9,k+1));
                
            end

        end
        
        function sen = checkSensorUpdate(t,sen,k)
            
            if k ~= t.N  
                                
                % Increment GPS and set update flag to true if GPS update
                % should occur
                if mod(k*t.dt,1/sen.gps.rr) == 0        % update sensor reading at rr
                    sen.gps.k = sen.gps.k+1;            % increment sensor
                    sen.gps.update = 1;                 % set update flag to true
                    sen.gps.lastUpdateTime = t.now;     % set the last time the sensor updated
                else
                    sen.gps.update = 0;                 % set update flag to false
                end

                % Increment IMU and set update flag to true if IMU update
                % should occur
                if mod(k*t.dt,1/sen.imu.rr) == 0    % update sensor reading at rr
                    sen.imu.k = sen.imu.k+1;        % increment sensor
                    sen.imu.update = 1;             % set update flag to true
                    sen.imu.lastUpdateTime = t.now; % set the last time the sensor updated
                else
                    sen.imu.update = 0;             % set update flag to false
                end
            end
            
        end
        
        function sen = updateSensors(t,sim,sen,obj,k)
                        
            if k ~= t.N  
                        
                % Add noise to GPS states to simulate sensor data if
                % GPS update should occur
                if sen.gps.update == 1
                    sen.gps.xm(1:2,sen.gps.k) = ...
                        awgn(sim.ym(1:2,k),sen.gps.snr);
                end

                % Add noise to IMU states to simulate sensor data if
                % IMU update should occur
                if sen.imu.update == 1
                    sen.imu.xr(3,sen.imu.k) =...
                        awgn(sim.yr(3,k),sen.imu.snr);
                    sen.imu.xr(6,sen.imu.k) =...
                        awgn(sim.yr(6,k),sen.imu.snr);
                    sen.imu.xr(7:8,sen.imu.k) =...
                        awgn(sim.yr(7:8,k),sen.imu.snr);
                end
                
                % Transform GPS from map frame to robot frame. Use sensed
                % orientation for transformation.
                if sen.gps.update == 1
                    sen.gps.xr(:,sen.gps.k) =...
                        obj.tf_m2r(sen.gps.xm(:,sen.gps.k),...
                        sen.imu.xr(3,sen.imu.k),...
                        0,...
                        0);
                end
                
                % Transform IMU from robot frame to map frame. Use sensed
                % orientation for transformation. Note: this is a poor
                % transformation due for acceleration terms.
                if sen.imu.update == 1
                    sen.imu.xm(:,sen.imu.k) = ...
                        obj.tf_r2m(sen.imu.xr(:,sen.imu.k),...
                        sen.imu.xr(3,sen.imu.k),...
                        sen.imu.xr(6,sen.imu.k),...
                        0);
                end
            end
        end
               
        function caEst = caEst(caEst,t,sen,obj,k)

            if k ~= t.N
                                                        
                % Define state estimator plant
                A = [...
                    1 0 0 caEst.dt 0 0 0 0 0;...
                    0 1 0 0 caEst.dt 0 0 0 0;...
                    0 0 1 0 0 caEst.dt 0 0 0;...
                    0 0 0 1 0 0 caEst.dt 0 0;...
                    0 0 0 0 1 0 0 caEst.dt 0;...
                    0 0 0 0 0 1 0 0 caEst.dt;...
                    0 0 0 0 0 0 1 0 0;...
                    0 0 0 0 0 0 0 1 0;...
                    0 0 0 0 0 0 0 0 1];
                B = [zeros(9,3)];

                % State estimator setup
                state.x = caEst.xm(:,caEst.k);              % last estimate
                state.P = caEst.Px(:,:,caEst.k);            % last covariance matrix
                state.Q = caEst.Q;                          % noise covariance (process)
                state.R = caEst.R;                          % noise covariance (measurement)
                state.H = caEst.H;                          % observation matrix
                state.z = [...                              % measurement
                    sen.gps.xr(1,sen.gps.k);...
                    sen.gps.xr(2,sen.gps.k);...
                    sen.imu.xr(3,sen.imu.k);...
                    zeros(2,1);...
                    sen.imu.xr(6:8,sen.imu.k);...
                    zeros(1,1)];
                state.u = [0;0;0];                      % control input (don't give kalman filter knowledge about thruster inputs)

                % Discrete Kalman filter
                state.A = A;
                state.B = B;
                state = obj.kalmanf(state);

                % Store state estimate
                caEst.xr(:,caEst.k+1) = state.x;
                caEst.Px(:,:,caEst.k+1) = state.P;
                caEst.Lx(:,:,caEst.k+1) = state.K;
                caEst.xm(:,caEst.k+1) = obj.tf_r2m(caEst.xr(:,caEst.k+1),...
                    caEst.xr(3,caEst.k+1),...
                    caEst.xr(6,caEst.k+1),...
                    0);

                % Increment tracking k variable
                caEst.k = caEst.k+1;

            end

        end
        
        function mcaEst = mcaEstMeas(mcaEst,t,sen)
            % Measurement pre-processing layer for modified constant
            % acceleration estimator
            
            % Set current variance gain matrix to identity
            mcaEst.varGain(:,:,mcaEst.k) = eye(9,9);     % variance gain matrix
                        
            % Check if a GPS update occured based on last known sensor update
            if mcaEst.gpsLastUpdateTime ~= sen.gps.lastUpdateTime
                
                % Set estimator last known sensor update to last known
                % sensor update
                mcaEst.gpsLastUpdateTime = sen.gps.lastUpdateTime;
                
                % Update the estimator gps sensor array
                mcaEst.gpsArray(:,2:end) = mcaEst.gpsArray(:,1:end-1);    
                mcaEst.gpsArray(:,1) = sen.gps.xr(:,sen.gps.k);
                
            end
            
            % Check if an IMU update occured based on last known sensor update
            if mcaEst.imuLastUpdateTime ~= sen.imu.lastUpdateTime
                
                % Set estimator last known sensor update to last known
                % sensor update
                mcaEst.imuLastUpdateTime = sen.imu.lastUpdateTime;
                
                % Update the estimator gps sensor array
                mcaEst.imuArray(:,2) = mcaEst.imuArray(:,1);        % note that this array MUST be 2 columns
                mcaEst.imuArray(:,1) = sen.imu.xr(:,sen.imu.k);
                
            end
            
            % Sliding covariance profiling gain. Two options: decrease
            % confidence in measurement noise covariance, R, or increase
            % confidence in process noise covariance, Q. The former is more
            % pragmatically correct. Decreasing confidence in measurement
            % noise covariance involves scaling the default covariance
            % between itself and an arbitrarily large number.
            gainDegree = 5;                         % degree of variance scaling (exponential, e.g. gain = 10^gpsGainDegree)
            gpsSmoothTimeEnd = (1/sen.gps.rr)*0.1;  % when the smoothing time should end (squared decrease of variance to 1)
            gpsScaleTimeEnd = (1/sen.gps.rr)*0.3;   % when the scaling time should end (exponential growth of variance from 1)
            
            % Calculate time since last update (TSLU)
            gpsTSLU = t.now - mcaEst.gpsLastUpdateTime;     % gps time since last update
            imuTSLU = t.now - mcaEst.imuLastUpdateTime;     % imu time since last update
            
            % Calculate GPS R-gain
            maxVarGain = 10^gainDegree;
            if (gpsTSLU >= 0 && gpsTSLU < gpsSmoothTimeEnd)                     % smoothing portion
                slope = gainDegree/gpsSmoothTimeEnd;
                gpsVarGain = 10^(gainDegree-slope*gpsTSLU);
            elseif (gpsTSLU >= gpsSmoothTimeEnd && gpsTSLU <= gpsScaleTimeEnd)  % scaling portion
                eConst = log(maxVarGain)/(gpsScaleTimeEnd-gpsSmoothTimeEnd);
                gpsVarGain = exp(eConst*(gpsTSLU-gpsSmoothTimeEnd));
            else 
                gpsVarGain = maxVarGain;
            end
            
            % Not going to perform sliding covariance profiling on IMU,
            % since the estimator needs to significantly temporally
            % outresolve the IMU refresh rate to be of use. Most estimators
            % will run at a speed similar to the refresh rate of the IMU,
            % which will cause problems
            imuVarGain = 1;     % this always trusts the IMU measurements as valid
            
%             % Uncomment for debugging...
%             disp(['time now: ',num2str(t.now),'; gpsTSLU: ',num2str(gpsTSLU),'; gpsObsGain: ',num2str(gpsVarGain)]);
%             disp(['time now: ',num2str(t.now),'; imuTSLU: ',num2str(imuTSLU),'; imuObsGain: ',num2str(imuVarGain)]);
%             disp(' ')
            
            % Linear position measurements
            mcaEst.meas.gps(1:2,mcaEst.k) = mcaEst.gpsArray(1:2,1);
            mcaEst.meas.all(1:2,mcaEst.k) = mcaEst.meas.gps(1:2,mcaEst.k);
            mcaEst.varGain(1:2,:,mcaEst.k) = [...
                1 0 0 0 0 0 0 0 0;...
                0 1 0 0 0 0 0 0 0]*gpsVarGain;
            
            % Angular position measurements
            mcaEst.meas.imu(3,mcaEst.k) = mcaEst.imuArray(3,1);
            mcaEst.meas.all(3,mcaEst.k) = mcaEst.meas.imu(3,mcaEst.k);
            mcaEst.varGain(3,:,mcaEst.k) = [...
                0 0 1 0 0 0 0 0 0]*imuVarGain;
            
            % Linear velocity measurements (forward-difference from GNSS
            % and trapezoidal method from IMU)
            useGps = 1;     % set to zero to ignore GPS forward difference
            useImu = 1;     % set to zero to ignore IMU trapezoidal method
            
            % Takes ONLY the forward-difference of GPS
            if (useGps == 1 && useImu == 0)                     % GPS only
                mcaEst.meas.gps(4:5,mcaEst.k) = (1/((1/sen.gps.rr)*...              % linear velocity via forward difference of GPS
                    mcaEst.gps.n))*...
                    (mcaEst.gpsArray(1:2,1)-...
                    mcaEst.gpsArray(1:2,mcaEst.gps.n));
                mcaEst.meas.all(4:5,mcaEst.k) = mcaEst.meas.gps(4:5,mcaEst.k);
                mcaEst.varGain(4:5,:,mcaEst.k) = [...                       % scale the measurement noise covariances by gpsVarGain
                    0 0 0 1 0 0 0 0 0;...
                    0 0 0 0 1 0 0 0 0]*gpsVarGain;
            
            % Takes ONLY the trapezoidal method of IMU
            elseif (useGps == 0 && useImu == 1)                                     % linear velocity via trapezoidal method of IMU
                if (mcaEst.k-1 == 0)
                    mcaEst.meas.imu(4:5,mcaEst.k) =...
                        (1/2)*(mcaEst.imuArray(7:8,2)+...
                        mcaEst.imuArray(7:8,1))*...
                        (1/sen.imu.rr);
                    mcaEst.meas.all(4:5,mcaEst.k) = mcaEst.meas.imu(4:5,mcaEst.k);
                else
                    mcaEst.meas.imu(4:5,mcaEst.k) =...
                        mcaEst.xr(4:5,mcaEst.k-1)+...
                        (1/2)*(mcaEst.imuArray(7:8,2)+...
                        mcaEst.imuArray(7:8,1))*...
                        (1/sen.imu.rr);
                    mcaEst.meas.all(4:5,mcaEst.k) = mcaEst.meas.imu(4:5,mcaEst.k);
                end
                mcaEst.varGain(4:5,:,mcaEst.k) = [...                       % scale the measurement noise covariances by imuVarGain
                    0 0 0 1 0 0 0 0 0;...
                    0 0 0 0 1 0 0 0 0]*imuVarGain;
                
            % Takes both forward-difference of GPS and trapezoidal method
            % of IMU (typically desired)
            elseif (useGps == 1 && useImu == 1)
                
                % Calculate forward-difference of GPS measurements
                mcaEst.meas.gps(4:5,mcaEst.k) = (1/((1/sen.gps.rr)*mcaEst.gps.n))*...
                    (mcaEst.gpsArray(1:2,1)-...
                    mcaEst.gpsArray(1:2,mcaEst.gps.n));
                
                % Calculate trapezoidal method of IMU measurements
                if (mcaEst.k-1 == 0)
                    mcaEst.meas.imu(4:5,mcaEst.k) =...
                        (1/2)*(mcaEst.imuArray(7:8,2)+...
                        mcaEst.imuArray(7:8,1))*...
                        (1/sen.imu.rr);
                else
                    mcaEst.meas.imu(4:5,mcaEst.k) =...
                        mcaEst.xr(4:5,mcaEst.k-1)+...
                        (1/2)*(mcaEst.imuArray(7:8,2)+...
                        mcaEst.imuArray(7:8,1))*...
                        (1/sen.imu.rr);
                end
                
                % Calculate the combined weight of the GPS and IMU based on
                % the current variances of both
                gpsVarGain = gpsVarGain*5;              % scale gain (larger value trusts GPS less, and vice-versa).
                gpsVarWeight = (1/gpsVarGain)/...
                    ((1/gpsVarGain)+(1/imuVarGain));
                imuVarWeight = (1/imuVarGain)/...
                    ((1/gpsVarGain)+(1/imuVarGain));
                mcaEst.meas.all(4:5,mcaEst.k) = mcaEst.meas.gps(4:5,mcaEst.k)*...
                    gpsVarWeight+mcaEst.meas.imu(4:5,mcaEst.k)*imuVarWeight;
                mcaEst.varGain(4:5,:,mcaEst.k) = [...
                    0 0 0 1 0 0 0 0 0;...
                    0 0 0 0 1 0 0 0 0]*...
                    (gpsVarGain*gpsVarWeight+imuVarGain*imuVarWeight);
                
            else
                mcaEst.meas.all(4:5,mcaEst.k) = [0;0];
                mcaEst.varGain(4:5,:,mcaEst.k) = [...
                    0 0 0 1 0 0 0 0 0;...
                    0 0 0 0 1 0 0 0 0];
            end
            
            % Angular velocity measurements
            mcaEst.meas.imu(6,mcaEst.k) = mcaEst.imuArray(6,1);
            mcaEst.meas.all(6,mcaEst.k) = mcaEst.meas.imu(6,mcaEst.k);
            mcaEst.varGain(6,:,mcaEst.k) = [...                                 % trust IMU linear position by setting observation matrix to one
                0 0 0 0 0 1 0 0 0]*imuVarGain;
            
            % Linear acceleration measurements
            mcaEst.meas.imu(7:8,mcaEst.k) = mcaEst.imuArray(7:8,1);
            mcaEst.meas.all(7:8,mcaEst.k) = mcaEst.meas.imu(7:8,mcaEst.k);
            mcaEst.varGain(7:8,:,mcaEst.k) = [...                               % trust IMU linear position by setting observation matrix to one
                0 0 0 0 0 0 1 0 0;...
                0 0 0 0 0 0 0 1 0]*imuVarGain;

        end
                
        function mcaEst = mcaEst(mcaEst,t,obj,k)

            if k ~= t.N

                % Executes when estimator should update
                if mod(k*t.dt,mcaEst.dt) == 0

                    % Define state estimator plant
                    A = [...
                        1 0 0 mcaEst.dt 0 0 0 0 0;...
                        0 1 0 0 mcaEst.dt 0 0 0 0;...
                        0 0 1 0 0 mcaEst.dt 0 0 0;...
                        0 0 0 1 0 0 mcaEst.dt 0 0;...
                        0 0 0 0 1 0 0 mcaEst.dt 0;...
                        0 0 0 0 0 1 0 0 mcaEst.dt;...
                        0 0 0 0 0 0 1 0 0;...
                        0 0 0 0 0 0 0 1 0;...
                        0 0 0 0 0 0 0 0 1];
                    B = [zeros(9,3)];

                    % State estimator setup
                    state.x = mcaEst.xr(:,mcaEst.k);                    % last estimate
                    state.P = mcaEst.Px(:,:,mcaEst.k);                  % last covariance matrix
                    state.Q = mcaEst.Q;                                 % process noise covariance
                    state.R = mcaEst.R*mcaEst.varGain(:,:,mcaEst.k);    % measurement noise covariance
                    state.H = mcaEst.H;                                 % measurement matrix
                    state.z = mcaEst.meas.all(:,mcaEst.k);              % meausrements
                    state.u = [0;0;0];                                  % control input vector (don't give kalman filter knowledge about thruster inputs)
                    
                    % Discrete Kalman filter
                    state.A = A;
                    state.B = B;
                    state = obj.kalmanf(state);
                    
                    % Store state estimate
                    mcaEst.xr(:,mcaEst.k+1) = state.x;
                    mcaEst.xm(:,mcaEst.k+1) = obj.tf_r2m(mcaEst.xr(:,mcaEst.k+1),...
                        mcaEst.xr(3,mcaEst.k+1),...
                        mcaEst.xr(6,mcaEst.k+1),...
                        0);
                    mcaEst.Px(:,:,mcaEst.k+1) = state.P;
                    mcaEst.Lx(:,:,mcaEst.k+1) = state.K;

                    % Increment tracking k variable
                    mcaEst.k = mcaEst.k+1;

                % Executes when estimator should not update (this happens
                % when the simulation executes more rapidly than the
                % estimator)
                else

                    % Do nothing

                end

            end

        end
        
        function s = kalmanf(s)

            % set defaults for absent fields:
            if ~isfield(s,'x'); s.x = nan*z; end
            if ~isfield(s,'P'); s.P = nan; end
            if ~isfield(s,'z'); error('Observation vector missing'); end
            if ~isfield(s,'u'); s.u = 0; end
            if ~isfield(s,'A'); s.A = eye(length(s.x)); end
            if ~isfield(s,'B'); s.B = 0; end
            if ~isfield(s,'Q'); s.Q = zeros(length(s.x)); end
            if ~isfield(s,'R'); error('Observation covariance missing'); end
            if ~isfield(s,'H'); s.H = eye(length(s.x)); end

            % Prediction for state vector and covariance:  
            s.x = s.A*s.x+s.B*s.u;
            s.P = s.A*s.P*s.A'+s.Q;

            % Compute Kalman gain factor:
            s.K = s.P*s.H'*inv(s.H*s.P*s.H'+s.R);

            % Correction based on observation:
            s.x = s.x + s.K*(s.z-s.H*s.x);
            s.P = s.P - s.K*s.H*s.P;

        end
    
    end      
        
end