classdef Cartesian
    %Cartesian Express in cartesian coordinates.
    %   Detailed explanation goes here
    
    properties
        X = 0;
        Y = 0;
        Z = 0;
        
        % Matrix: [X, Y, Z]
        XYZ;
    end
    
    properties(Access = private)
        MatrixNRows = 1;
    end
    
    methods
        % Constructor
        function self = Cartesian(~)
            
        end
        
        function matrix = xyz2Matrix(self)
            matrix = [self.X, self.Y, self.Z];
        end
        
        function self = matrix2XYZ(self, matrix, nRows)
            %matrix2XYZ Fills X, Y, Z from matrix columns.
            % Matrix format is [X, Y, Z].
            % nRows: # of rows in matrix.
            
            % If nRows is omitted...
            if(nargin <= 2)
                nRows = 1;
            end
            
            % Preallocate X, Y, Z if size of data does change.
            if(nRows ~= self.MatrixNRows)
                self = preallocateXYZ();
                self.MatrixNRows = nRows;
            end
            
            % Fill X, Y, Z with specified dimensions.
            self.X(1:nRows, 1) = matrix(1:nRows, 1);
            self.Y(1:nRows, 1) = matrix(1:nRows, 2);
            self.Z(1:nRows, 1) = matrix(1:nRows, 3);
        end
    end
    
    methods(Access = private)
        function self = preallocateXYZ(self)
            % preallocateXYZ Use this to preallocate X, Y, Z ahead of time.
            self.X = zeros(self.MatrixNRows, 1);
            self.Y = zeros(self.MatrixNRows, 1);
            self.Z = zeros(self.MatrixNRows, 1);
        end
    end
end

