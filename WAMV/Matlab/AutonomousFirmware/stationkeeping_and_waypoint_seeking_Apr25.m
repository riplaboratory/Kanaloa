function main()

% insert HOSTNAME
%type "hostname in terminal to find
%getenv('ROS_MASTER_URI')
%getenv('ROS_HOSTNAME')
%getenv('ROS_IP')
rosinit

%Connect Sensors to CODE
GPSSub = rossubscriber('/fix', 'sensor_msgs/NavSatFix');
IMUSub = rossubscriber('/imu/data', 'sensor_msgs/Imu');

% ================================================ ====================

%initialize variables
GPSmsg = receive(GPSSub);
IMUmsg = receive(IMUSub);

goal_lat = GPSmsg.Latitude;
goal_lon = GPSmsg.Longitude;
lla = [goal_lat, goal_lon, 0];
llo = [goal_lat,goal_lon];
goal_flatearth_pos = lla2flat(lla, llo, 0, 0);
goal_vars = [goal_flatearth_pos(1), goal_flatearth_pos(2), 0];

IMUmsg = receive(IMUSub);
Ox = IMUmsg.Orientation.X;
Oy = IMUmsg.Orientation.Y;
Oz = IMUmsg.Orientation.Z;
Ow = IMUmsg.Orientation.W;
quat = [Ox, Oy, Oz, Ow];
[yaw, pitch, roll] = quat2angle(quat);
Final_Goal_Angle = yaw;
goal_Angle = Final_Goal_Angle;

%goal_vars = [currentX, currentY, 0]; % Hold position and heading
%dataX = zeros(100,1);
%dataY = zeros(100,1);
%dataLat = zeros(100,1);
%dataLon = zeros(100,1);
running = true;

Velocity_X = 0;
Velocity_Y = 0;
RelativePosition_X = 0;
RelativePosition_Y = 0;
Error_X = 0;
Error_Y = 0;
Error_Angle = 0;
PrevError_X = 0;
PrevError_Y = 0;
PrevError_Angle = 0;
DError_X = 0;
DError_Y = 0;
DError_Angle = 0;
IError_X = 0;
IError_Y = 0;
IError_Angle = 0;

% These three values control the PID loops for X, Y and rotational PID -- 
% one set of values for all three; may need to expand later.
kP = 8.0;
kI = 0.02;
kD = 0.0;

% kRotational determines how much weight is given to trying to keep the 
% boat straight vs trying to keep it in the right X,Y position. 
% Higher values = more weight on rotational control.
% Lower values = more weight on position control
% 1.0 = Even weight for both.
kRotational = 0.50; %1 is too low; 10-20 may be ok
kSway = -1.0; %designed to be one; set to 0 to supress
kSurge = 1.0; %designed to be one; set to zero to supress

DesiredThrust_X = 0;
DesiredThrust_Y = 0;

% Output to publish (placeholder local variables)
ThrustOutput_Q1 = 0;
ThrustOutput_Q2 = 0;
ThrustOutput_Q3 = 0;
ThrustOutput_Q4 = 0;

% Define thresholds for switching behavior between waypoint navigation and
% stationkeeping.
StationkeepingThreshold = 10; %in Meters!!!
WaypointSeekingThreshold = 15; %in Meters!!!
Stationkeeping = false;

while(running)
   
    % Get new sensor data for position and heading
    GPSmsg = receive(GPSSub);
    IMUmsg = receive(IMUSub);
    
    flag = GPSmsg.Status.Status;
    lat = GPSmsg.Latitude;
    lon = GPSmsg.Longitude;
    lla = [lat, lon, 0];
    flatearth_pos = lla2flat(lla, llo, 0, 0);
    
    %   ===================================================================
    %  TODO: make RelativePosition relative to a specified goal state and
    %  not just the position the WAM-V is in when the program starts!!
    %   ===================================================================
    
    RelativePosition_X = flatearth_pos(1);
    RelativePosition_Y = flatearth_pos(2);
 
    %   ===================================================================
    %   <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><
    %   
    %                   Get the heading X and Y from the IMU
    %                 And put it into Heading_X and Heading_Y
    % Karla/Thomas: put orientation X,Y into heading X,Y
    %   <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><
    %   ===================================================================
    
    % orientation x,y,z,w
    Ox = IMUmsg.Orientation.X;
    Oy = IMUmsg.Orientation.Y;
    Oz = IMUmsg.Orientation.Z;
    Ow = IMUmsg.Orientation.W;

    % angular velocity x,y,z
    Ax = IMUmsg.AngularVelocity.X;
    Ay = IMUmsg.AngularVelocity.Y;
    Az = IMUmsg.AngularVelocity.Z;

    % linear acceleration x,y,z
    Lx = IMUmsg.LinearAcceleration.X;
    Ly = IMUmsg.LinearAcceleration.Y;
    Lz = IMUmsg.LinearAcceleration.Z;
    
    % Heading from the IMU:
    % Heading_Angle_Radians = 0;
    % Heading_X = Ox;
    % Heading_Y = Oy;
    
    quat = [Ox, Oy, Oz, Ow];
    [yaw, pitch, roll] = quat2angle(quat);
    
    % ASSUMING yaw is rotation about Z (X is forward)
    Heading_X = -cos(yaw);
    Heading_Y = -sin(yaw);
    
    fprintf('yaw: %f,  pitch: %f, roll: %f\n',yaw,pitch,roll);
    fprintf('\n')
   
    %======================================================================
    % Waypoint-seeking behavior switch, BJ Tix 25 April 2017
    %======================================================================
    
    % Determine the mode we should be in
    % if far from the target, then we are in waypoint seeking mode.
    % if close to the target, then we are in stationkeeping mode.
    distance_to_target = sqrt((RelativePosition_X * RelativePosition_X) + (RelativePosition_Y * RelativePosition_Y));
    
    % Change the gains to modify the behavior for the mode we are in.
    % in stationkeeping, the integral gain is used as is strafing

    if((false == Stationkeeping) && (distance_to_target < StationkeepingThreshold))
        Stationkeeping = true;
        kP = 8.0;
        kI = 0.02;
        kD = 0.05;        

        kRotational = 0.50; %1 is too low; 10-20 may be ok
        kSway = -1.0; %designed to be one; set to 0 to supress
        kSurge = 1.0; %designed to be one; set to zero to supress
        
        % While stationkeeping, try to keep the pre-specified heading.
        goal_Angle = Final_Goal_Angle;
        
        if(false == Stationkeeping)
            IError_X = 0;
            IError_Y = 0;
            IError_Angle = 0;
        end  
    end
    
    % In waypoint seeking, no integral gain is used and strafing is
    % supressed.


    if(Stationkeeping && (0 < WaypointSeekingThreshold) && (WaypointSeekingThreshold < distance_to_target))
        Stationkeeping = false;
        kP = 12.0;
        kI = 0.0;
        kD = 0.0;        
        kRotational = 8.0; %1 is too low; 10-20 may be ok
        kSway = 0.0; %designed to be one; set to 0 to supress
        kSurge = 1.0; %designed to be one; set to zero to supress
        
        % While waypoint seeking, always head towards the goal.
        goal_Angle = atan2(RelativePosition_Y, RelativePosition_X);
    end
     
    % Calculate Errors for PID controller ------------------------------
    % Proportional Error
    Error_X = -RelativePosition_X;
    Error_Y = -RelativePosition_Y;
    Error_Angle = goal_Angle - yaw;
   

if(Error_X < -12)

 Error_X = -12;
end
if(Error_Y < -12)
  Error_Y = -12;
end

if(12 < Error_X)
   Error_X = 12;
end
if(12<Error_Y)
    Error_Y=12;
end 
    while(Error_Angle < -3.14159)
		Error_Angle = Error_Angle + 6.283;
    end
    
    while(3.14159 < Error_Angle)
		Error_Angle = Error_Angle - 6.283;
    end
    
    % Error = [Error_X Error_Y];
    % disp('Error');
    % disp(Error);
    
    fprintf('error X: %f, error y: %f\n',Error_X, Error_Y);
    fprintf('\n')
    
    % Derivative Error
    DError_X = Error_X - PrevError_X;
    DError_Y = Error_Y - PrevError_Y;
    DError_Angle = Error_Angle - PrevError_Angle;
    
    % Integral Error
    IError_X = IError_X + Error_X;
    IError_Y = IError_Y + Error_Y;
    IError_Angle = IError_Angle + Error_Angle;
    
    % Update Previous Error for use with future derivative error
    PrevError_X = Error_X;
    PrevError_Y = Error_Y;
    PrevError_Angle = Error_Angle;
    
    % Calculate Thrust generated by each error component----------------
    % SINGLE set of gains for each thruster and for X and Y.
    DesiredThrust_X = (kP*Error_X) + (kI*IError_X) + (kD*DError_X);
    DesiredThrust_Y = (kP*Error_Y) + (kI*IError_Y) + (kD*DError_Y);
    DesiredThrust_Angular = (kP*Error_Angle) + (kI*IError_Angle) + (kD*DError_Angle);

    % Convert these thrust values into values for each thruster:
    %
    % Basically some of the corners need to go forwards and some
    % backwards based on these values, so figure out which corner
    % corresponds to which direction. Apply the thrust for each
    % thruster based on this. The value will either be positive or
    % negative.
    %
    %
    % MAP:
    % UPDATE 17 April
    %
    %                \ 
    %  Q3------------Q2
    %  /  :    :      
    %     :    :               ===> Direction of travel
    %  \  :    :      
    %  Q4------------Q1
    %                /
    %
    %   / and \ are positive thrust directions. All positive = boat
    %   moves forward.
    %
    % PROBLEM!!!!!!
    %
    % Without heading data, how will we know which direction each
    % thruster is facing???????
    %
    % Forward:          Q1+ Q2+ Q3+ Q4+
    % Backwards:        Q1- Q2- Q3- Q4-
    % Strafe Left:      Q1+ Q2- Q3+ Q4-   (up in picture)
    % Strafe Right:     Q1- Q2+ Q3- Q4+   (down in picture)
    % Turn Left:        Q1+ Q2- Q3- Q4+
    % Turn Right:       Q1- Q2+ Q3+ Q4-
    %
    % Knowing how to move forwards, backwards, left and right is great,
    % but unless the boat is facing due North that doesn't correspond
    % to X and Y.
    %
    % SO!!!! We need the IMU integrated into this; to take a reading
    % from the IMU in order to figure out what direction the WAM-V is
    % facing in order to know which direction to move to get back to
    % the home position.
    %
    % ASSUME that Heading can be properly assigned from the
    % IMU.
    
    % With the heading data, we're going to use a trig transform to come up
    % with the proportion of each of the four cardinal relative directions
    % we need to use.
    
    % Take a dot product of the current heading with the desired direction
    % of travel.
    % surge = thrust vector (dot) heading vector
    % The ratio of motion which must be in the forward direction.
    % will be one if the boat is already facing the correct direction.
    surge = (DesiredThrust_X * Heading_X) + (DesiredThrust_Y * Heading_Y);
    % surge = dot(DesiredThrust_X, Heading_X) + dot(DesiredThrust_Y, Heading_Y);
    
    % Negative values of ratio_forward mean to move backwards instead.
    Surge_Q1 = surge;
    Surge_Q2 = surge;
    Surge_Q3 = surge;
    Surge_Q4 = surge;
    
    % To calculate the sway direction motion we need to get the
    % perpendicular vector for the heading:
    heading_normal_X = -Heading_Y;
    heading_normal_Y = Heading_X;
    
    % Take a dot product again to get the Sway thrust:
    sway = (DesiredThrust_X * heading_normal_X) + (DesiredThrust_Y * heading_normal_Y);
    % sway = dot(DesiredThrust_X, heading_normal_X) + dot(DesiredThrust_Y, heading_normal_Y);
    
    Sway_Q1 = sway;
    Sway_Q2 = -sway;
    Sway_Q3 = sway;
    Sway_Q4 = -sway;
    
    % The dot products will have automatically handled the scaling and
    % negative values. Sketch it out for confirmation if you like.
    % Now we need to add the two together to get the total thrust for each
    % thruster.
    % UPDATE 17 April 2017 ALSO include rotational control
    ThrustOutput_Q1 = (kSurge * Surge_Q1) + (kSway * Sway_Q1) - (kRotational * DesiredThrust_Angular);
    ThrustOutput_Q2 = (kSurge * Surge_Q2) + (kSway * Sway_Q2) + (kRotational * DesiredThrust_Angular);
    ThrustOutput_Q3 = (kSurge * Surge_Q3) + (kSway * Sway_Q3) + (kRotational * DesiredThrust_Angular);
    ThrustOutput_Q4 = (kSurge * Surge_Q4) + (kSway * Sway_Q4) - (kRotational * DesiredThrust_Angular);
    
    fprintf('Thrustouput 1-4: %f,%f,%f,%f\n',ThrustOutput_Q1,ThrustOutput_Q2,ThrustOutput_Q3,ThrustOutput_Q4);
    fprintf('\n')
    
    % Convert thrust to motor controller duty cycle
    Motor_Controller_Q1 = ThrustOutput_Q1;
    Motor_Controller_Q2 = ThrustOutput_Q2;
    Motor_Controller_Q3 = ThrustOutput_Q3;
    Motor_Controller_Q4 = ThrustOutput_Q4;
    
    if(50 < Motor_Controller_Q1)
        Motor_Controller_Q1 = 50;
    end
    
    if(50 < Motor_Controller_Q2)
        Motor_Controller_Q2 = 50;
    end
    
    if(50 < Motor_Controller_Q3)
        Motor_Controller_Q3 = 50;
    end
    
    if(50 < Motor_Controller_Q4)
        Motor_Controller_Q4 = 50;
    end
    
    Motor_Controller_Q1 = Motor_Controller_Q1 + 50.0;
    Motor_Controller_Q2 = Motor_Controller_Q2 + 50.0;
    Motor_Controller_Q3 = Motor_Controller_Q3 + 50.0;
    Motor_Controller_Q4 = Motor_Controller_Q4 + 50.0;
    
    if Motor_Controller_Q1 < 0 
        Motor_Controller_Q1 = 0;
    end
    
    if Motor_Controller_Q2 < 0
        Motor_Controller_Q2 = 0;
    end
    
    if Motor_Controller_Q3 < 0
        Motor_Controller_Q3 = 0;
    end
    
    if Motor_Controller_Q4 < 0
        Motor_Controller_Q4 = 0;
    end
    
    %  ================================================================
    %  The follow will tranfer our motor command outputs to a ROS topic
    
    % Creates ROS topic under '/motor_qN' and setting vaiable type
    mQ1_pub = rospublisher('/motor_q1', 'std_msgs/UInt16');   % create Matlab publisher to Q1 Arduino
    mQ2_pub = rospublisher('/motor_q2', 'std_msgs/UInt16');   % create Matlab publisher to Q2 Arduino
    mQ3_pub = rospublisher('/motor_q3', 'std_msgs/UInt16');   % create Matlab publisher to Q3 Arduino
    mQ4_pub = rospublisher('/motor_q4', 'std_msgs/UInt16');   % create Matlab publisher to Q4 Arduino

    % Converting ROS topic to MATLAB variable
    m_msg1 = rosmessage(mQ1_pub);
    m_msg2 = rosmessage(mQ2_pub);
    m_msg3 = rosmessage(mQ3_pub);
    m_msg4 = rosmessage(mQ4_pub);
    
    % Inputting our controller outputs to ROS Publisher variable
    m_msg1.Data = round(Motor_Controller_Q1, 0);
    m_msg2.Data = round(Motor_Controller_Q2, 0);
    m_msg3.Data = round(Motor_Controller_Q3, 0);
    m_msg4.Data = round(Motor_Controller_Q4, 0);
 
    % debug
    if(Stationkeeping)
        fprintf('Mode: Stationkeeping \n');
    else
        fprintf('Mode: Waypoint Seeking \n');
    end
    
    fprintf('m_msg 1-4: %f,%f,%f,%f \n',Motor_Controller_Q1,Motor_Controller_Q2,Motor_Controller_Q3,Motor_Controller_Q4);
    fprintf('\n');
    fprintf('Goal Heading: %f radians, %f degrees, X: %f Y: %f', goal_Angle, goal_Angle * 180 / pi, cos(goal_Angle), sin(goal_Angle));
 
    % Publish to ROS TOPIC 
    send(mQ1_pub,m_msg1);
    send(mQ2_pub,m_msg2);
    send(mQ3_pub,m_msg3);
    send(mQ4_pub,m_msg4);
    
end
% END ROS
rosshutdown;
end
