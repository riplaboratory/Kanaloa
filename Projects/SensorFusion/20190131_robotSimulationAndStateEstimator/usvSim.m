classdef usvSim
        
    methods (Static)
        
        function [t,lp,ic,sen] = simDefaults()
            % Sets simulation variables to their defaults.  Can change the
            % values in this function OR edit the structure yourself.
            
            % USER INPUT: time properties
            t.dt = 0.001;   % simulation time step [s]
            t.tend = 50;    % simulation end time [s]

            % USER INPUT: USV lumped parameters
            lp.m = 225;     % mass/inertia [kg]
            lp.I = 100;     % rotational moment of inertia [kg*m^2]
            lp.bxr = 25;    % drag in the surge (robot frame along x) direction [N*s/m]
            lp.byr = 300;   % drag in the sway (robot frame along y) direction [N*s/m]
            lp.btzr = 300;  % rotational drag in the yaw (robot frame about z) direction [N*s/m]

            % USER INPUT: initial conditions in the map frame
            ic.x0 = 0;      % initial position (robot frame along x) [m] 
            ic.y0 = 0;      % initial position (robot frame along y) [m]
            ic.tz0 = 0;     % initial angle (robot frame about z) [rad]
            ic.dx0 = 0;     % initial velocity (robot frame along x) [m/s] 
            ic.dy0 = 0;     % initial velocity (robot frame along y) [m/s] 
            ic.dtz0 = 0;    % initial angular velocity (robot about z) [rad/s] 

            % USER INPUT: sensor setup
            sen.gps.rr = 2;     % refresh rate of GPS sensor [Hz]
            sen.gps.snr = 2;    % signal-to-noise ratio of GPS sensor [ ]
            sen.imu.rr = 100;   % refresh rate of IMU sensor [Hz]
            sen.imu.snr = 40;   % signal-to-noise ratio of IMU sensor [ ]
            
            % Dependent variables
            t.N = round(t.tend/t.dt);   % total number of time steps [ ]
        
        end
        
        function caest = caEstDefaults()
            % Constant acceleration state-transition model default values.
            
            % USER INPUT: time properties
            caest.dt = 0.005;   % estimator time step [s]
            
            % USER INPUT: process noise covariance
            caest.Q = [...
                0.01 0 0 0 0 0 0 0 0;...    % [m]
                0 0.01 0 0 0 0 0 0 0;...    % [m]
                0 0 0.01 0 0 0 0 0 0;...    % [rad]
                0 0 0 1 0 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 1 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 0 0.01 0 0 0;...    % [rad/s]
                0 0 0 0 0 0 1 0 0;...       % [m/s^2]
                0 0 0 0 0 0 0 1 0;...       % [m/s^2]
                0 0 0 0 0 0 0 0 1];         % [rad/s^2] (not measured)
    
            % USER INUT: measurement noise covariance       
            caest.R = [...
                0.1 0 0 0 0 0 0 0 0;...     % [m]
                0 0.1 0 0 0 0 0 0 0;...     % [m]
                0 0 0.1 0 0 0 0 0 0;...     % [rad]
                0 0 0 1 0 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 1 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 0 0.1 0 0 0;...     % [rad/s]
                0 0 0 0 0 0 0.1 0 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0.1 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0 1];         % [rad/s] (not measured)

            % USER INPUT: observation matrix 
            caest.H = [...
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
        
        function ddest = ddEstDefaults()
            % Discrete derivative state-transition model default values.
            
            % USER INPUT: time properties
            ddest.dt = 0.005;   % estimator time step [s]
            
            % USER INPUT: process noise covariance
            ddest.Q = [...
                0.001 0 0 0 0 0 0 0 0;...    % [m]
                0 0.001 0 0 0 0 0 0 0;...    % [m]
                0 0 0.01 0 0 0 0 0 0;...    % [rad]
                0 0 0 1 0 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 1 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 0 0.01 0 0 0;...    % [rad/s]
                0 0 0 0 0 0 1 0 0;...       % [m/s^2]
                0 0 0 0 0 0 0 1 0;...       % [m/s^2]
                0 0 0 0 0 0 0 0 1];         % [rad/s^2] (not measured)
    
            % USER INUT: measurement noise covariance       
            ddest.R = [...
                0.1 0 0 0 0 0 0 0 0;...     % [m]
                0 0.1 0 0 0 0 0 0 0;...     % [m]
                0 0 0.1 0 0 0 0 0 0;...     % [rad]
                0 0 0 1 0 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 1 0 0 0 0;...       % [m/s] (not measured)
                0 0 0 0 0 0.1 0 0 0;...     % [rad/s]
                0 0 0 0 0 0 0.1 0 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0.1 0;...     % [m/s^2]
                0 0 0 0 0 0 0 0 1];         % [rad/s] (not measured)

            % USER INPUT: observation matrix 
            ddest.H = [...
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
        
        function [sim,sen] = simOutputSetup(t,sen)
            
            % State, output, and input vectors
            sim.xr = zeros(9,t.N);  % state vector (robot reference frame)
            sim.yr = zeros(9,t.N);  % output vector (robot reference frame)
            sim.xm = zeros(9,t.N);  % state vector (map reference frame)
            sim.ym = zeros(9,t.N);  % output vector (map reference frame)
            sim.u = zeros(3,t.N);   % input vector (control inputs act on robot frame)
            
            % Sensor data vectors
            sen.gps.xr = zeros(9,t.N);  % gps data vector (robot reference frame)
            sen.gps.xm = zeros(9,t.N);  % gps data vector (map reference frame)
            sen.imu.xr = zeros(9,t.N);  % imu data vector (robot reference frame)
            sen.imu.xm = zeros(9,t.N);  % imu data vector (map reference frame)
            
            % Sensor update tracker
            sen.gps.dt = 0;     % gps update time tracker
            sen.imu.dt = 0;     % imu update time tracker
            
        end
        
        function caest = caEstOutputSetup(caest,t)
           
            % Instantiate estimate matrices
            caest.xm = zeros(9,t.N);        % state estimate in map frame
            caest.xr = zeros(9,t.N);        % state estimate in robot frame
            caest.Lx = zeros(9,9,t.N);      % kalman gain matrix
            caest.Px = zeros(9,9,t.N);      % covariance matrix
            
        end
        
        function ddest = ddEstOutputSetup(ddest,t)
           
            % Instantiate estimate matrices
            ddest.xm = zeros(9,t.N);        % state estimate in map frame
            ddest.xr = zeros(9,t.N);        % state estimate in robot frame
            ddest.Lx = zeros(9,9,t.N);      % kalman gain matrix
            ddest.Px = zeros(9,9,t.N);      % covariance matrix
            ddest.ym = zeros(9,t.N);        % state estimate + observer (output) in map frame
            ddest.yr = zeros(9,t.N);        % state estimate + observer (output) in robot frame
            ddest.Ly = zeros(9,9,t.N);      % kalman gain matrix
            ddest.Py = zeros(9,9,t.N);      % covariance matrix
            
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
        
        function vecM = tf_r2m(vecR,tz)
            
            vecM = [...
                cos(tz) -sin(tz) 0 0 0 0 0 0 0;...
                sin(tz) cos(tz) 0 0 0 0 0 0 0;...
                0 0 1 0 0 0 0 0 0;...
                0 0 0 cos(tz) -sin(tz) 0 0 0 0;...
                0 0 0 sin(tz) cos(tz) 0 0 0 0;...
                0 0 0 0 0 1 0 0 0;...
                0 0 0 0 0 0 cos(tz) -sin(tz) 0;...
                0 0 0 0 0 0 sin(tz) cos(tz) 0;...
                0 0 0 0 0 0 0 0 1]*vecR;
            
        end
        
        function vecR = tf_m2r(vecM,tz)
        
            vecR = [...
                cos(tz) -sin(tz) 0 0 0 0 0 0 0;...
                sin(tz) cos(tz) 0 0 0 0 0 0 0;...
                0 0 1 0 0 0 0 0 0;...
                0 0 0 cos(tz) -sin(tz) 0 0 0 0;...
                0 0 0 sin(tz) cos(tz) 0 0 0 0;...
                0 0 0 0 0 1 0 0 0;...
                0 0 0 0 0 0 cos(tz) -sin(tz) 0;...
                0 0 0 0 0 0 sin(tz) cos(tz) 0;...
                0 0 0 0 0 0 0 0 1]'*vecM;
        
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
            sim.xr(:,1) = obj.tf_m2r(sim.xm(:,1),ic.tz0);
            sim.yr(:,1) = obj.tf_m2r(sim.xm(:,1),ic.tz0);
            
        end
        
        function sim = controller(sim,k)

            if k > 1

                % Decide on desired thruster components
                sim.u(1,k) = sim.u(1,k-1)+0.0025;   % force input in surge direction
                sim.u(2,k) = sim.u(2,k-1)+0.0025;   % force input in sway direction
                sim.u(3,k) = sim.u(3,k-1)+0.0025;   % torque input in yaw direction

            end

        end
        
        function sim = runSim(t,p,sim,obj,k)

            F = eye(9)+p.A*t.dt;
            G = p.B*t.dt;                                           % discrete control input matrix
            H = p.C;                                                % discrete output matrix
            J = p.D;                                                % discrete feedthrough (direct transmission) matrix
            if k ~= t.N

                % Simulate plant using discrete Euler integration
                sim.xr(:,k+1) = F*sim.yr(:,k)+...
                    G*[sim.u(1,k);sim.u(2,k);sim.u(3,k)];     % state matrix integration solution
                sim.yr(:,k+1) = H*sim.xr(:,k+1)+...
                    J*[sim.u(1,k);sim.u(2,k);sim.u(3,k)];     % state observer matrix integration solution
                
                % Convert robot frame to map frame
                sim.xm(:,k+1) = obj.tf_r2m(sim.xr(:,k+1),sim.xr(3,k+1));
                sim.ym(:,k+1) = obj.tf_r2m(sim.yr(:,k+1),sim.yr(3,k+1));
                                
            end

        end
        
        function [sim,sen] = updateSensors(t,sim,sen,obj,k)
                        
            if k ~= t.N  
                        
                % GPS
                if mod(k*t.dt,1/sen.gps.rr) == 0                                % update sensor reading at rr
                    sen.gps.xm(1:2,k+1) = awgn(sim.xm(1:2,k),sen.gps.snr);      % add noise
                else
                    sen.gps.xm(1:2,k+1) = sen.gps.xm(1:2,k);
                end

                % IMU
                if mod(k*t.dt,1/sen.imu.rr) == 0                                % update sensor reading at rr
                    sen.imu.xr(3,k+1) = awgn(sim.xr(3,k),sen.imu.snr);          % add noise
                    sen.imu.xr(6,k+1) = awgn(sim.xr(6,k),sen.imu.snr);          % add noise
                    sen.imu.xr(7:9,k+1) = awgn(sim.xr(7:9,k),sen.imu.snr);      % add noise
                else
                    sen.imu.xr(3,k+1) = sen.imu.xr(3,k);
                    sen.imu.xr(6,k+1) = sen.imu.xr(6,k);
                    sen.imu.xr(7:9,k+1) = sen.imu.xr(7:9,k);
                end

                % Transform GPS from map frame to robot frame
                sen.gps.xr(:,k+1) = obj.tf_m2r(sen.gps.xm(:,k+1),sen.imu.xr(3,k+1));    % note that the transformantion uses sensed orientation instead of simulated orientation
                
                % Transform IMU from robot frame to map frame
                sen.imu.xm(:,k+1) = obj.tf_r2m(sen.imu.xr(:,k+1),sen.imu.xr(3,k+1));    % note that the transformantion uses sensed orientation instead of simulated orientation
                
            end

        end
        
        function caest = caEst(caest,t,sen,obj,k)

            if k ~= t.N

                % Only update filter at estimator time steps
                if mod(k*t.dt,caest.dt) == 0

                    % Define state estimator plant
                    A = [...
                        1 0 0 caest.dt 0 0 0 0 0;...
                        0 1 0 0 caest.dt 0 0 0 0;...
                        0 0 1 0 0 caest.dt 0 0 0;...
                        0 0 0 1 0 0 caest.dt 0 0;...
                        0 0 0 0 1 0 0 caest.dt 0;...
                        0 0 0 0 0 1 0 0 caest.dt;...
                        0 0 0 0 0 0 1 0 0;...
                        0 0 0 0 0 0 0 1 0;...
                        0 0 0 0 0 0 0 0 1];
                    B = [zeros(9,3)];

                    % State estimator setup
                    state.x = caest.xm(:,k);                % last estimate
                    state.P = caest.Px(:,:,k);              % last covariance matrix
                    state.Q = caest.Q;                      % noise covariance (process)
                    state.R = caest.R;                      % noise covariance (measurement)
                    state.H = caest.H;                      % observation matrix
                    state.z = [sen.gps.xm(1:2,k+1);...      % measurement
                        sen.imu.xm(3,k+1);...
                        zeros(2,1);...
                        sen.imu.xm(6:8,k+1);...
                        zeros(1,1)];
                    state.u = [0;0;0];                      % control input (don't give kalman filter knowledge about thruster inputs)

                    % Discrete Kalman filter
%                     state.A = eye(9)+A*(caest.dt);
%                     state.B = B*(caest.dt);
                    state.A = A;
                    state.B = B;
                    state = obj.kalmanf(state);

                    % Store state estimate
                    caest.xm(:,k+1) = state.x;
                    caest.Px(:,:,k+1) = state.P;
                    caest.Lx(:,:,k+1) = state.K;
                    caest.xr(:,k+1) = obj.tf_m2r(caest.xm(:,k+1),caest.xm(3,k+1));

                else

                    caest.xm(:,k+1) = caest.xm(:,k);
                    caest.Px(:,:,k+1) = caest.Px(:,:,k);
                    caest.Lx(:,:,k+1) = caest.Lx(:,:,k);
                    caest.xr(:,k+1) = caest.xr(:,k);

                end

            end

        end
                
        function [ddest,sen] = ddEst(ddest,t,sen,obj,k)

            if k ~= t.N
                                        
                % Determine whether estimator should update
                if mod(k*t.dt,ddest.dt) == 0
                    
                    % Instantiate Kalman matrices
                    meas = zeros(9,1);      % measurements
                    measCov = eye(9,9);     % measurement noise covariance
                    
                    % Determine if GPS sensor updated
                    if sen.gps.xm(1,k+1) == sen.gps.xm(1,k)
                    
                        % No update on GPS sensor
                        gpsUpdate = false;                      % set GPS update flag to false
                        gpsDt = ddest.dt;                       % set time step in plant to estimator time step
                        sen.gps.dt = sen.gps.dt + ddest.dt;     % increment GPS time step tracker
                        measCov(1:2,:) = [...                   % distrust GPS linear position measurement noise covariance (set it large)
                            1 0 0 0 0 0 0 0 0;...
                            0 1 0 0 0 0 0 0 0]*1E6;
                    
                    else
                        
                        % GPS update
                        gpsUpdate = true;                       % set GPS update flag to true
                        gpsDt = sen.gps.dt;                     % set time step in plant to gps time step tracker
                        sen.gps.dt = 0;                         % reset gps time step tracker
                        meas(1:2,:) = sen.gps.xm(1:2,k+1);      % add GPS positions to measurement matrix
                        measCov(1:2,:) = ddest.R(1:2,:);        % trust GPS linear position measurement noise covariance (set it to user default)
                    
                    end
                    
                    % Determine time step for IMU sensor
                    if sen.imu.xm(1,k+1) == sen.imu.xm(1,k)
                        
                        % No update on IMU sensor, increment time step
                        imuUpdate = false;                      % set IMU update flag to false
                        imuDt = ddest.dt;                       % set time step in plant to estimator time step
                        sen.imu.dt = sen.imu.dt + ddest.dt;     % increment imu time step tracker
                        measCov(3,:) = [...                     % distrust IMU anglar position measurement noise covariance (set it large)
                            0 0 1 0 0 0 0 0 0]*1E6;
                        measCov(6,:) = [...                     % distrust IMU anglar position measurement noise covariance (set it large)
                            0 0 0 0 0 1 0 0 0]*1E6;
                        measCov(7:8,:) = [...                   % distrust IMU linear acceleartion measurement noise covariance (set it large)
                            0 0 0 0 0 0 1 0 0;...
                            0 0 0 0 0 0 0 1 0]*1E6;
                        
                    else
                        
                        % IMU update, save time step, add measurements
                        imuUpdate = true;                       % set IMU update flag to true
                        imuDt = sen.imu.dt;                     % set time step in plant to gps time step tracker
                        sen.imu.dt = 0;                         % reset gps time step tracker
                        meas(3,:) = sen.imu.xm(3,k+1);          % add IMU angular position to measurement matrix
                        meas(6,:) = sen.imu.xm(6,k+1);          % add IMU angular velocity to measurement matrix
                        meas(7:8,:) = sen.imu.xm(7:8,k+1);      % add IMU linear accelerations to measurement matrix
                        measCov(3,:) = ddest.R(3,:);            % trust IMU angular position measurement noise covariance (set it to user default)
                        measCov(6,:) = ddest.R(6,:);            % trust IMU angular veloicty measurement noise covariance (set it to user default)
                        measCov(7:8,:) = ddest.R(7:8,:);        % trust IMU linear acceleration measurement noise covariance (set it to user default)
                                                
                    end
                    
                    % Define state estimator plant
%                     A = [...
%                         1 0 0 ddest.dt 0 0 0 0 0;...
%                         0 1 0 0 ddest.dt 0 0 0 0;...
%                         0 0 1 0 0 ddest.dt 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 1 0 0 ddest.dt;...
%                         0 0 0 0 0 0 1 0 0;...
%                         0 0 0 0 0 0 0 1 0;...
%                         0 0 0 0 0 0 0 0 1];
                    A = [...
                        1 0 0 0 0 0 0 0 0;...
                        0 1 0 0 0 0 0 0 0;...
                        0 0 1 0 0 0 0 0 0;...
                        0 0 0 0 0 0 0 0 0;...
                        0 0 0 0 0 0 0 0 0;...
                        0 0 0 0 0 1 0 0 0;...
                        0 0 0 0 0 0 1 0 0;...
                        0 0 0 0 0 0 0 1 0;...
                        0 0 0 0 0 0 0 0 1];
                    B = [zeros(9,3)];
                    C1 = [...
                        1 0 0 0 0 0 0 0 0;...
                        0 1 0 0 0 0 0 0 0;...
                        0 0 1 0 0 0 0 0 0;...
                        1/gpsDt 0 0 0 0 0 0 0 0;...
                        0 1/gpsDt 0 0 0 0 0 0 0;...
                        0 0 0 0 0 1 0 0 0;...
                        0 0 0 0 0 0 1 0 0;...
                        0 0 0 0 0 0 0 1 0;...
                        0 0 0 0 0 0 0 0 1];
                    C2 = [...
                        0 0 0 0 0 0 0 0 0;...
                        0 0 0 0 0 0 0 0 0;...
                        0 0 0 0 0 0 0 0 0;...
                        -1/gpsDt 0 0 0 0 0 0 0 0;...
                        0 -1/gpsDt 0 0 0 0 0 0 0;...
                        0 0 0 0 0 0 0 0 0;...
                        0 0 0 0 0 0 0 0 0;...
                        0 0 0 0 0 0 0 0 0;...
                        0 0 0 0 0 0 0 0 0];                    
                    D = [zeros(9,3)];

                    % State estimator setup
                    state.x = ddest.ym(:,k);        % last estimate
                    state.P = ddest.Py(:,:,k);      % last covariance matrix
                    state.Q = ddest.Q;              % process noise covariance
                    state.R = measCov;              % measurement noise covariance
                    state.H = ddest.H;              % observation matrix
                    state.z = meas;                 % meausrement matrix
                    state.u = [0;0;0];              % control input (don't give kalman filter knowledge about thruster inputs)

                    % Discrete Kalman filter
                    state.A = A;
                    state.B = B;
                    state = obj.kalmanf(state);

                    % Store state estimate
                    ddest.xm(:,k+1) = state.x;
                    ddest.xr(:,k+1) = obj.tf_m2r(ddest.xm(:,k+1),ddest.xm(3,k+1));
                    ddest.Px(:,:,k+1) = state.P;
                    ddest.Lx(:,:,k+1) = state.K;

                    % State estimator + observer setup
                    output.x1 = ddest.xm(:,k+1);            % last estimate (from state estimate)
                    output.x2 = ddest.xm(:,k);              % last estimate (from state estimate)
                    output.P = ddest.Px(:,:,k+1);           % last covariance matrix
                    output.Q = ddest.Q;                     % noise covariance (process)
                    output.R = measCov;                     % noise covariance (measurement)
                    output.H = ddest.H;                     % observation matrix
                    output.z = meas;                        % measurement matrix
                    output.u = [0;0;0];                     % control input (don't give kalman filter knowledge about thruster inputs)

                    % Discrete Kalman filter
                    output.A1 = C1;
                    output.A2 = C2;
                    output.B = D;
                    output = obj.kalmanf2(output);

                    % Store output
                    ddest.ym(:,k+1) = output.x;
                    ddest.yr(:,k+1) = obj.tf_m2r(ddest.ym(:,k+1),ddest.ym(3,k+1));
                    ddest.Py(:,:,k+1) = output.P;
                    ddest.Ly(:,:,k+1) = output.K;
                    
                else
                    
                    % Estimator not updating at this time, set all
                    % estimator values to equal
                    ddest.xm(:,k+1) = ddest.xm(:,k);
                    ddest.xr(:,k+1) = ddest.xr(:,k);
                    ddest.Px(:,:,k+1) = ddest.Px(:,:,k);
                    ddest.Lx(:,:,k+1) = ddest.Lx(:,:,k);
                    ddest.ym(:,k+1) = ddest.ym(:,k);
                    ddest.yr(:,k+1) = ddest.yr(:,k);
                    ddest.Py(:,:,k+1) = ddest.Py(:,:,k);
                    ddest.Ly(:,:,k+1) = ddest.Ly(:,:,k);                    
    
                end
                
            end

        end
        
%         function ddest = ddEst(ddest,t,sen,obj,k)
% 
%             if k ~= t.N
% 
%                 % Only update filter at estimator time steps
%                 if mod(k*t.dt,ddest.dt) == 0
%                                         
%                     
%                     % Define state estimator plant
%                     A = [...
%                         1 0 0 ddest.dt 0 0 0 0 0;...
%                         0 1 0 0 ddest.dt 0 0 0 0;...
%                         0 0 1 0 0 ddest.dt 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 1 0 0;...
%                         0 0 0 0 0 0 0 1 0;...
%                         0 0 0 0 0 0 0 0 1];
%                     B = [zeros(9,3)];
%                     C1 = [...
%                         1 0 0 0 0 0 0 0 0;...
%                         0 1 0 0 0 0 0 0 0;...
%                         0 0 1 0 0 0 0 0 0;...
%                         1/ddest.dt 0 0 0 0 0 0 0 0;...
%                         0 1/ddest.dt 0 0 0 0 0 0 0;...
%                         0 0 1/ddest.dt 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 1 0 0;...
%                         0 0 0 0 0 0 0 1 0;...
%                         0 0 0 0 0 0 0 0 1];
%                     C2 = [...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         -1/ddest.dt 0 0 0 0 0 0 0 0;...
%                         0 -1/ddest.dt 0 0 0 0 0 0 0;...
%                         0 0 -1/ddest.dt 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 0 0 0;...
%                         0 0 0 0 0 0 0 0 0];
%                     D = [zeros(9,3)];
% 
%                     % State estimator setup
%                     state.x = ddest.ym(:,k);                % last estimate
%                     state.P = ddest.Py(:,:,k);              % last covariance matrix
%                     state.Q = ddest.Q;                      % noise covariance (process)
%                     state.R = ddest.R;                      % noise covariance (measurement)
%                     state.H = ddest.H;                      % observation matrix
%                     state.z = [sen.gps.xm(1:2,k+1);...      % measurement
%                         sen.imu.xm(3,k+1);...
%                         zeros(2,1);...
%                         sen.imu.xm(6:8,k+1);...
%                         zeros(1,1)];
%                     state.u = [0;0;0];                      % control input (don't give kalman filter knowledge about thruster inputs)
% 
%                     % Discrete Kalman filter
%                     state.A = eye(9)+A*(ddest.dt);
%                     state.B = B*(ddest.dt);
%                     state = obj.kalmanf(state);
% 
%                     % Store state estimate
%                     ddest.xm(:,k+1) = state.x;
%                     ddest.xr(:,k+1) = obj.tf_m2r(ddest.xm(:,k+1),ddest.xm(3,k+1));
%                     ddest.Px(:,:,k+1) = state.P;
%                     ddest.Lx(:,:,k+1) = state.K;
%                     
%                     % State estimator + observer setup
%                     output.x1 = ddest.xm(:,k+1);             % last estimate (from state estimate)
%                     output.x2 = ddest.xm(:,k);               % last estimate (from state estimate)
%                     output.P = ddest.Px(:,:,k+1);            % last covariance matrix
%                     output.Q = ddest.Q;                      % noise covariance (process)
%                     output.R = ddest.R;                      % noise covariance (measurement)
%                     output.H = ddest.H;                      % observation matrix
%                     output.z = [sen.gps.xm(1:2,k+1);...      % measurement
%                         sen.imu.xm(3,k+1);...
%                         zeros(2,1);...
%                         sen.imu.xm(6:8,k+1);...
%                         zeros(1,1)];
%                     output.u = [0;0;0];                      % control input (don't give kalman filter knowledge about thruster inputs)
% 
%                     % Discrete Kalman filter
%                     output.A1 = C1;
%                     output.A2 = C2;
%                     output.B = D;
%                     output = obj.kalmanf2(output);
% 
%                     % Store output
%                     ddest.ym(:,k+1) = output.x;
%                     ddest.yr(:,k+1) = obj.tf_m2r(ddest.ym(:,k+1),ddest.ym(3,k+1));
%                     ddest.Py(:,:,k+1) = output.P;
%                     ddest.Ly(:,:,k+1) = output.K;
% 
%                 else
% 
%                     ddest.xm(:,k+1) = ddest.xm(:,k);
%                     ddest.xr(:,k+1) = ddest.xr(:,k);
%                     ddest.Px(:,:,k+1) = ddest.Px(:,:,k);
%                     ddest.Lx(:,:,k+1) = ddest.Lx(:,:,k);
%                     ddest.ym(:,k+1) = ddest.ym(:,k);
%                     ddest.yr(:,k+1) = ddest.yr(:,k);
%                     ddest.Py(:,:,k+1) = ddest.Py(:,:,k);
%                     ddest.Ly(:,:,k+1) = ddest.Ly(:,:,k);
% 
%                 end
% 
%             end
% 
%         end
        
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
        
        function s = kalmanf2(s)

            % set defaults for absent fields:
            if ~isfield(s,'x1'); s.x1 = nan*z; end
            if ~isfield(s,'x2'); s.x2 = nan*z; end
            if ~isfield(s,'P'); s.P = nan; end
            if ~isfield(s,'z'); error('Observation vector missing'); end
            if ~isfield(s,'u'); s.u = 0; end
            if ~isfield(s,'A1'); s.A1 = eye(length(s.x)); end
            if ~isfield(s,'A2'); s.A2 = eye(length(s.x)); end
            if ~isfield(s,'B'); s.B = 0; end
            if ~isfield(s,'Q'); s.Q = zeros(length(s.x)); end
            if ~isfield(s,'R'); error('Observation covariance missing'); end
            if ~isfield(s,'H'); s.H = eye(length(s.x)); end
            
            % Prediction for state vector and covariance:  
            s.x = (s.A1*s.x1)+(s.A2*s.x2)+s.B*s.u;
%             s.P = (s.A1+s.A2)*s.P*(s.A1+s.A2)'+s.Q;
            s.P = s.A1*s.P*s.A1'+s.A2*s.P*s.A2'+s.Q;
            
            % Compute Kalman gain factor:
            s.K = s.P*s.H'*inv(s.H*s.P*s.H'+s.R);

            % Correction based on observation:
            s.x = s.x + s.K*(s.z-s.H*s.x);
            s.P = s.P - s.K*s.H*s.P;

        end
    
    end      
        
end