classdef IMUAngle < Angle
    %IMUAngle Gets orientation data from IMU
    
    properties
        
    end
    
    properties(Access = private)
        % GPS topic subscriber
        IMUSub;
        % Data from subscriber
        IMUMsg;
        Orientation;
        AngularVelocity;
    end
    
    methods
        % Constructor: topic subscription goes here and
        % registered to this object.
        function self = IMUAngle(imu_sub)
            self.IMUSub = imu_sub;
        end
        
        function self = getAngles(self)
            %getAngles Get angular data and convert to useful format.
            
            self = self.receiveFromIMU();
            self = self.updateYPR();
        end
    end
    
    methods(Access = private)
        function self = receiveFromIMU(self)
            %receiveFromIMU Get [all] kinematic data from IMU.
            
            % Get the data from subscriber.
            self.IMUMsg = receive(self.IMUSub);
            
            % Get orientation component only.
            self.Orientation = self.IMUMsg.Orientation;
            
            % Get angular velocity components only.
            self.AngularVelocity = self.IMUMsg.AngularVelocity;
            
            % Write orientations.
            self.X = self.Orientation.X;
            self.Y = self.Orientation.Y;
            self.Z = self.Orientation.Z;
            self.W = self.Orientation.W;
            
            % Write angular velocities.
            self.XaxisVelocity = self.AngularVelocity.X;
            self.YaxisVelocity = self.AngularVelocity.Y;
            self.ZaxisVelocity = self.AngularVelocity.Z;
        end
    end
end

