classdef IMUAcceleration < Position
    %IMUAcceleration Takes acceleration data from IMU
    %   Detailed explanation goes here
    
    properties
    end
    
    properties(Access = private)
        IMUSub;
        IMUMsg;
        LinearAcceleration;
    end
    
    methods
        % Constructor: topic subscription goes here and
        % registered to this object.
        function self = IMUAcceleration(imu_sub)
            self.IMUSub = imu_sub;
        end
        
        function self = getAccelerations(self)
            self = self.receiveFromIMU();
        end
    end
    
    methods(Access = private)
        function self = receiveFromIMU(self)
            %receiveFromIMU Get [all] kinematic data from IMU
            
            % Get the data from subscriber
            self.IMUMsg = receive(self.IMUSub);
            % Get orientation component only
            self.LinearAcceleration = self.IMUMsg.Orientation;
            self.X = self.LinearAcceleration.X;
            self.Y = self.LinearAcceleration.Y;
            self.Z = self.LinearAcceleration.Z;
        end
    end
end

