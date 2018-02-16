classdef Angle
    %Angle Contains and processes angular positions, velocities.
    %   Stores individual components
    %   Converts into single quaternion array
    %   Converts quaternions to axis angles and stores it
    
    properties
       % Axis angles.
       Yaw = 0;
       Pitch = 0;
       Roll = 0;
       % Axis angles.
       XaxisVelocity = 0;
       YaxisVelocity = 0;
       ZaxisVelocity = 0;
    end
    
    properties(Access = protected)
       % Quaternion components.
       % Each has to be column matrix.
       X = 0;
       Y = 0;
       Z = 0;
       W = 0;
    end
    
    methods
        function matrix = toMatrix(self)
            %toMatrix creates a matrix of the variable
            % arr columns: X, Y, Z, W.
            matrix = [self.X, self.Y, self.Z, self.W];
        end
        
        function self = updateYPR(self)
            %updateYPR converts and updates quaternions to axis angles.
            [self.Yaw, self.Pitch, self.Roll] = quat2angle( self.toMatrix() );
        end
    end
    
end

