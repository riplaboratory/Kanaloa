classdef GPS
    %GPS Get location data and convert to Cartesian coordinates.
    %   Dependenc(ies): Cartesian
    
    properties
        Flag = 0;
        
        Longitude = 0;
        GoalLongitude = 0;
        Latitude = 0;
        GoalLatitude = 0;
        Altitude = 0;
        
        % Relative position object.
        RelPosition;
    end
    
    properties(Access = private)
        % GPS topic subscriber
        GPSSub;
        % Data from subscriber
        GPSMsg;
        
        % Reference longitude/latitude in an array (no altitude).
        % [degrees, degrees]
        LLO = [0, 0];
        % Reference altitude (meters)
        Href = 0;
        % Store longitude/latitude/altitude in an array.
        % [degrees, degrees, meters]
        LLA = [0, 0, 0];
        % # of degrees rotation of flat Earth coord sys from NWSE coord sys
        Psio = 0;
    end
    
    methods
        function self = GPS(gps_sub)
            self.GPSSub = gps_sub;
            
            % Create RelPosition.
            self.RelPosition = Cartesian();
        end
        
        function self = getCoords(self)
            %getCoords Update long/lat/alt and X, Y, Z.
            
            self = self.receiveFromGPS();
            self.LLA = [self.Longitude, self.Latitude, self.Altitude];
            
            % Get flat-earth coords and store to XYZ matrix.
            % LLA could be a multi-row matrix. It will return multi-row
            % matrix.
            self.RelPosition.XYZ = lla2flat(self.LLA, self.LLO, self.Psio, self.Href);
            self.RelPosition = self.RelPosition.matrix2XYZ(self.RelPosition.XYZ);
        end
        
        function self = setReferenceCoords(self)
            %setReferenceCoords Register reference long/lat into GPS obj
            
            self.GoalLongitude = self.Longitude;
            self.GoalLatitude = self.Latitude;
            self.LLO = [self.GoalLongitude, self.GoalLatitude];
        end
    end
    
    methods(Access = private)
        function self = receiveFromGPS(self)
            %receiveFromGPS Get long/lat data.
            
            % Get GPS data.
            self.GPSMsg = receive(self.GPSSub);
            
            % Flag
            self.Flag = self.GPSMsg.Status.Status;
            
            % Write longitude/latitude.
            self.Longitude = self.GPSMsg.Longitude;
            self.Latitude = self.GPSMsg.Latitude;
        end
    end
end

