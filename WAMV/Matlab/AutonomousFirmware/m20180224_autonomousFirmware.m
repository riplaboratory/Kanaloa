function m20180224_autonomousFirmware()

    % Initialize Matlab ROS node
    rosshutdown;
    rosinit;
    
    % Create topics that Matlab node will publish and subscribe to in ROS
    pub = createRosPublishers();
    sub = createRosSubscibers();
    
    % Wait for first topics to publish
    pause(1.25);
    
    % Instantiate (preallocate memory) for major structures
    [pose,goal,thrustCommands] = instantiateStructs();
    
    % Get map zero reference
    zeroPose = getZeroPose();
    
    % Set initial goal to current position of WAM-V
    goal = initialGoal(goal,zeroPose,pub);
        
    % Start control loop
    while (true)
        
        % Mandatory short pause (or else the callbacks won't execute!)
        pause(0.1);
        
        % Check the ROS goal topics, overwrite Matlab goals if changed
        goal = checkRosGoal(goal);
        
        % Update pose
        pose = updatePose(zeroPose,pose);
        
        % Controller (pick one!)
        thrustCommands = stationKeeping(goal,pose,pub,thrustCommands);
                
        % Display stuff
        displayStuff(goal,pose,thrustCommands);
        
    end

end

function pub = createRosPublishers()
    % Creates the relevant ROS publishers.

    % Create ROS topic (for publishing) in current instance of ROS
    pub.autoQ1 = rospublisher('/autoQ1', 'std_msgs/UInt16');
    pub.autoQ2 = rospublisher('/autoQ2', 'std_msgs/UInt16');
    pub.autoQ3 = rospublisher('/autoQ3', 'std_msgs/UInt16');
    pub.autoQ4 = rospublisher('/autoQ4', 'std_msgs/UInt16');
    pub.xGoal = rospublisher('/xGoalInput','std_msgs/Float32');
    pub.yGoal = rospublisher('/yGoalInput','std_msgs/Float32');
    pub.tzGoal = rospublisher('/tzGoalInput','std_msgs/Float32');
    
    % Display
    disp('Matlab ROS node publishers created...');
    
end

function sub = createRosSubscibers()
    % Creates the relevant ROS subscribers.

    try

        % Create ROS topic (for subscribing) in current instance of ROS
        sub.gps = rossubscriber('/fix',@gpsCallback);
        sub.gpsGS = rossubscriber('/groundGps',@gpsGSCallback);
        sub.imu = rossubscriber('/imu/data',@imuCallback);
        sub.xGoal = rossubscriber('/xGoalInput',@xGoalCallback);
        sub.yGoal = rossubscriber('/yGoalInput',@yGoalCallback);
        sub.tzGoal = rossubscriber('/tzGoalInput',@tzGoalCallrosback);
    
        % Display
        disp('Matlab ROS node scubscribers created...');
        
    catch
       
        disp(' ');
        disp('Cannot find topics, are you sure ROS is running???');
        disp(' ');
        
    end

end

function [pose,goal,thrustCommands] = instantiateStructs()
    % Preallocates/instantiates memory for important structure variables in
    % the code.  More for good bookeeping than speed...

    % Instantiate pose
    pose.x = 0;         % position
    pose.y = 0;
    pose.z = 0;
    pose.tx = 0;
    pose.ty = 0;
    pose.tz = 0;
    pose.dx = 0;        % velocity
    pose.dy = 0;
    pose.dz = 0;
    pose.dtx = 0;
    pose.dty = 0;
    pose.dtz = 0;
    pose.ddx = 0;       % acceleration
    pose.ddy = 0;
    pose.ddz = 0;
    pose.ddtx = 0;
    pose.ddty = 0;
    pose.ddtz = 0;

    % Instantiate goal
    goal.x = 0;
    goal.y = 0;
    
    % Instantiate trust commands
    thrustCommands.Q1 = 0;
    thrustCommands.Q2 = 0;
    thrustCommands.Q3 = 0;
    thrustCommands.Q4 = 0;
    
    % Display
    disp('Instantiating major structures...');
    
end

function zeroPose = getZeroPose()
    % Sets the zero point (pose) of the robot in the map frame.  When
    % differential GPS is working, set this zero to the ground station GPS
    % lat-lon.  For now setting this to the initial WAM-V position when
    % script runs.  
    
    % Instantiate ROS callback subsciber global variables
    global gpsLat gpsLon gpsAlt
    global gpsGSLat gpsGSLon gpsGSAlt
    global imutz
    
%     % Set inital pose based off sensor information
%     zeroPose.lat = gpsLat;       % latitude [deg]
%     zeroPose.lon = gpsLon;       % longitude [deg]
%     zeroPose.alt = gpsAlt;       % altitude [m]
%     zeroPose.tz = imutz;         % angle from north [rad]

    % Set inital pose based off sensor information
    zeroPose.lat = gpsGSLat;       % latitude [deg]
    zeroPose.lon = gpsGSLon;       % longitude [deg]
    zeroPose.alt = gpsGSAlt;       % altitude [m]
    zeroPose.tz = imutz;           % angle from north [rad]
        
    % Display
    disp('Grabbing zero pose...');
    
end

function goal = initialGoal(goal,zeroPose,pub)
    % Sets the initial goal of the WAM-V to the current position of the
    % WAM-V (so that it doesn't start moving when the code runs).

    % Get current position of WAM-V
    [x,y,~] = latlon2mm(zeroPose);

    % Set initial goal to current position of WAM-V
    goal.x = x;
    goal.y = y;
    goal.tz = zeroPose.tz;
        
    % Display
    disp('Setting initial waypoint goal to starting position');
    
end

function goal = checkRosGoal(goal)
    % Checks the /xGoalInput and yGoalInput topics in ROS, if they are
    % different than the current value in Matlab (goal.x and goal.y), then
    % overwrite the value in Matlab

    % Instantiate ROS callback subsciber global variables
    global xGoalInput yGoalInput tzGoalInput
        
    % Check ROS goal callbacks, if different from Matlab goal, overwrite
    if  goal.x ~= xGoalInput
        goal.x = xGoalInput;
    end
    if goal.y ~= yGoalInput
        goal.y = yGoalInput;
    end
    if goal.tz ~= tzGoalInput
        goal.tz = tzGoalInput;
    end
    
end

function pose = updatePose(zeroPose,pose)
    % Get sensor data and use it to update the pose of the robot.  Eventual
    % sensor fusion will insert itself in a big way here. 

    % Instantiate ROS callback subsciber global variables
    global imutx imuty imutz        % euler angular position
    global imudtx imudty imudtz     % euler angular velocity
    global imuddx imuddy imuddz     % euler linear acceleration
    
    % Convert lat-lon from GPS to meters
    [x,y,z] = latlon2mm(zeroPose);
    
    % Position
    pose.x = x;
    pose.y = y;
    pose.z = z;
    pose.tx = imutx;
    pose.ty = imuty;
    pose.tz = imutz;
    
    % Velocity
    pose.dx = 0;
    pose.dy = 0;
    pose.dz = 0;
    pose.dtx = imudtx;
    pose.dty = imudty;
    pose.dtz = imudtz;

    % Acceleration
    pose.ddx = imuddx;
    pose.ddy = imuddy;
    pose.ddz = imuddz;
    pose.ddtx = 0;
    pose.ddty = 0;
    pose.ddtz = 0;
    
end

function thrustCommands = stationKeeping(goal,pose,pub,thrustCommands)

    % EVENTUALLY, WE WILL WRITE NICE FUNCTIONS FOR ALL OF THIS, BUT RUSHING
    % FOR NOW. 
    
    % Calculate errors
    error.xDist2goal = goal.x-pose.x;
    error.yDist2goal = goal.y-pose.y;
    error.head2goal = goal.tz-pose.tz;
    error.dist2goal = sqrt((goal.x-pose.x)^2+(goal.y-pose.y)^2);
    error.pathHead2goal = atan2(sin(atan2(error.yDist2goal,...
        error.xDist2goal)-error.head2goal),cos(atan2(error.yDist2goal,...
        error.xDist2goal)-error.head2goal));   
    
    % Determine behavior
    % Behavior 1 = Outside approach radius, and not facing path heading.  Pure rotation to face goal.
    % Behavior 2 = Outside approach radius, and facing path heading.  Drive forward and control path heading to goal.  
    % Behavior 3 = Inside apprach radius, and not facing path heading.  Pure rotation to face goal.
    % Behavior 4 = Inside approach radius, and facing path heading. Drive forward slowly and control path heading to goal.  
    % Behavior 5 = Inside goal radius, but not facing path heading.  Pure rotation to face goal heading.
    % Behavior 6 = Inside goal radius, and facing path heading.  Full stop.
    
    % Instantiate behavior variable
    behavior = 0;
    
    % User variables
    approachRadius = 15;    % [m]
    goalRadius = 3;         % [m]
    pathArc = deg2rad(5);   % [rad]
    
    if error.dist2goal >= approachRadius
        if error.pathHead2goal >= pathArc
            behavior = 1;
        else
            behavior = 2; 
        end
    elseif error.dist2goal < approachRadius && error.dist2goal >= goalRadius
        if error.pathHead2goal >= pathArc
            behavior = 3;
        else
            behavior = 4;
        end
    else
        if error.head2goal >= pathArc
            behavior = 5;
        else
            behavior = 6;
        end
    end
    
    % PID gains
    kp.pos = 10;
    kp.rot = 15;
    kp.vel = 10; 
    kd.pos = 0.5;
    kd.rot = 0.3;
    kd.vel = 0.3;
    ki.pos = 0.1;
    ki.rot = 0.1;
    ki.vel = 0.1;
    
    % Controller output vector, u (all proportional PID controller for now...)
    if behavior == 1
        u.Q1.prop = kp.rot*error.pathHead2goal;
        u.Q2.prop = -kp.rot*error.pathHead2goal;
        u.Q3.prop = -kp.rot*error.pathHead2goal;
        u.Q4.prop = kp.rot*error.pathHead2goal;
    elseif behavior == 2
        u.Q1.prop = kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q2.prop = -kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q3.prop = -kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q4.prop = kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
    elseif behavior == 3
        u.Q1.prop = kp.rot*error.pathHead2goal;
        u.Q2.prop = -kp.rot*error.pathHead2goal;
        u.Q3.prop = -kp.rot*error.pathHead2goal;
        u.Q4.prop = kp.rot*error.pathHead2goal;
    elseif behavior == 4
        u.Q1.prop = kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q2.prop = -kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q3.prop = -kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q4.prop = kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
    elseif behavior == 5
        u.Q1.prop = kp.rot*error.head2goal;
        u.Q2.prop = -kp.rot*error.head2goal;
        u.Q3.prop = -kp.rot*error.head2goal;
        u.Q4.prop = kp.rot*error.head2goal;
    elseif behavior == 6
        u.Q1.prop = 0;
        u.Q2.prop = 0;
        u.Q3.prop = 0;
        u.Q4.prop = 0;
    end
       
    % Sum all output vectors
    thrustCommands.Q1 = u.Q1.prop;
    thrustCommands.Q2 = u.Q2.prop;
    thrustCommands.Q3 = u.Q3.prop;
    thrustCommands.Q4 = u.Q4.prop;
    
    % Limit control output vector
    if thrustCommands.Q1 >= 55
        thrustCommands.Q1 = 55;
    end
    if thrustCommands.Q1 <= -55
        thrustCommands.Q1 = -55;
    end
    if thrustCommands.Q2 >= 55
        thrustCommands.Q2 = 55;
    end
    if thrustCommands.Q2 <= -55
        thrustCommands.Q2 = -55;
    end
    if thrustCommands.Q3 >= 55
        thrustCommands.Q3 = 55;
    end
    if thrustCommands.Q3 <= -55
        thrustCommands.Q3 = -55;
    end
    if thrustCommands.Q4 >= 55
        thrustCommands.Q4 = 55;
    end
    if thrustCommands.Q4 <= -55
        thrustCommands.Q4 = -55;
    end
    
    % Map output vector, u, to 0 to 100 (what the topic wants)
   thrustCommands.Q1 = round(thrustCommands.Q1*(200/110),0);
   thrustCommands.Q2 = round(thrustCommands.Q2*(200/110),0);
   thrustCommands.Q3 = round(thrustCommands.Q3*(200/110),0);
   thrustCommands.Q4 = round(thrustCommands.Q4*(200/110),0);
    
    % Publish
    Q1msg = rosmessage(pub.autoQ1);
    Q2msg = rosmessage(pub.autoQ2);
    Q3msg = rosmessage(pub.autoQ3);
    Q4msg = rosmessage(pub.autoQ4);
    Q1msg.Data = thrustCommands.Q1;
    Q2msg.Data = thrustCommands.Q2;
    Q3msg.Data = thrustCommands.Q3;
    Q4msg.Data = thrustCommands.Q4;
    send(pub.autoQ1,Q1msg);
    send(pub.autoQ2,Q2msg);
    send(pub.autoQ3,Q3msg);
    send(pub.autoQ4,Q4msg);
    
end

function displayStuff(goal,pose,thrustCommands)
    % Display stuff to Matlab command window, or in terminal if SSHing.
    
    % Print information
    clc;
    fprintf('Current Goal [m]:      %.1f, %.1f, %.1f\n',goal.x,goal.y,goal.tz);
    fprintf('Current Position [m]:  %.1f, %.1f, %.1f\n',pose.x,pose.y,pose.z);
    fprintf('Current Heading [deg]: %.2f\n',rad2deg(pose.tz));
    fprintf('Thrusters [0 to 100]:  %u, %u, %u, %u,\n',...
        thrustCommands.Q1,thrustCommands.Q2,thrustCommands.Q3,thrustCommands.Q4);
    fprintf('\n');
    
end

function [x,y,z] = latlon2mm(zeroPose)
    % Takes the zero pose of of the GPS (in lat-lon), then looks at the
    % global variables for the current position of the robot (in lat-lon),
    % and uses this info to calculate the curren position of the robot in
    % [m x m].

    % Instantiate ROS callback subsciber global variables
    global gpsLat gpsLon gpsAlt     % latitude [deg], latitude [deg], and altitude [m]

    % Convert lattitude and longitude to meters in map
    lla = [gpsLat, gpsLon, gpsAlt];
    llo = [zeroPose.lat, zeroPose.lon];
    psio = rad2deg(zeroPose.tz);
    href = -zeroPose.alt;
    posmm = lla2flat(lla, llo, psio, href);
    
    % Because Matlab is silly and doesn't let lla2flat output directly to matrix...
    x = posmm(1);
    y = posmm(2);
    z = posmm(3);
    
end

function gpsCallback(~,message)

    global gpsLat gpsLon gpsAlt
    gpsLat = message.Latitude;
    gpsLon = message.Longitude;
    gpsAlt = message.Altitude;

end

function gpsGSCallback(~,message)

    global gpsGSLat gpsGSLon gpsGSAlt
    gpsGSLat = message.Latitude;
    gpsGSLon = message.Longitude;
    gpsGSAlt = message.Altitude;

end

function imuCallback(~,message)

    global imuQx imuQy imuQz imuQw  % quaternion angular position
    global imutx imuty imutz        % euler angular position
    global imudtx imudty imudtz     % euler angular velocity
    global imuddx imuddy imuddz     % euler linear acceleration
    
    imuQx = message.Orientation.X;
    imuQy = message.Orientation.Y;
    imuQz = message.Orientation.Z;
    imuQw = message.Orientation.W;
    [imutz,imuty,imutx] = quat2angle([imuQx,imuQy,imuQz,imuQw]);
    imudtx = message.AngularVelocity.X;
    imudty = message.AngularVelocity.Y;
    imudtz = message.AngularVelocity.Z;
    imuddx = message.LinearAcceleration.X;
    imuddy = message.LinearAcceleration.Y;
    imuddz = message.LinearAcceleration.Z;
    
end

function xGoalCallback(~,message)

    global xGoalInput
    xGoalInput = message.Data;

end

function yGoalCallback(~,message)

   global yGoalInput
   yGoalInput = message.Data;
   
end

function tzGoalCallback(~,message)

   global tzGoalInput
   tzGoalInput = message.Data;
   
end


%% DOODOO IN PANTS


function garbage()

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
