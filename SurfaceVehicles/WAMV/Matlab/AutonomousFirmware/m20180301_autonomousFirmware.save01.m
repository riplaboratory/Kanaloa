% Stuff to do
% not die if topics return NaN on position


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
    
    % Check if differential GPS can work
    dGps = dGpsCheck(sub);
    
    % Update pose
    pose = updatePose(pose,dGps);
        
    % Set initial goal to current position of WAM-V
    goal = initialGoal(goal,pose);
        
    % Start control loop
    while (true)
        
        % Mandatory short pause (or else the callbacks won't execute!)
        pause(0.1);
        
        % Check the ROS goal topics, overwrite Matlab goals if changed
        goal = checkRosGoal(goal);
        
        % Update pose
        pose = updatePose(pose,dGps);
        
        % Controller (pick one!)
        [thrustCommands,error] = stationKeeping(goal,pose,pub,thrustCommands);
                
        % Display stuff
        displayStuff(goal,pose,thrustCommands,error);
        
    end

end

function pub = createRosPublishers()
    % Creates the relevant ROS publishers.

    % Create ROS topic (for publishing) in current instance of ROS
    pub.autoQ1 = rospublisher('/autoQ1','std_msgs/Float64');
    pub.autoQ2 = rospublisher('/autoQ2','std_msgs/Float64');
    pub.autoQ3 = rospublisher('/autoQ3','std_msgs/Float64');
    pub.autoQ4 = rospublisher('/autoQ4','std_msgs/Float64');
    pub.xGoal = rospublisher('/xGoalInput','std_msgs/Float32');
    pub.yGoal = rospublisher('/yGoalInput','std_msgs/Float32');
    pub.tzGoal = rospublisher('/tzGoalInput','std_msgs/Float32');
    
    % Display
    disp('Matlab ROS node publishers created...');
    
end

function sub = createRosSubscibers()
    % Creates the relevant ROS subscribers.
    
    try
        sub.wamvGps = rossubscriber('/wamvGps',@wamvGpsCallback);
    catch
        disp(' ');
        disp('Cant communicate with wamvGPS! Code will throw error!');
    end
    
    try 
        sub.groundGps = rossubscriber('/groundGps',@groundGpsCallback);
    catch
        disp(' ');
        disp('Cant communicate with groundGPS! Differential GPS will not work!');
    end
    
    try
        sub.wamvImu = rossubscriber('/wamvImu/data',@imuCallback);
    catch
        disp(' ');
        disp('Cant communicate with wamvImu!  Code will throw error!');
    end

    try
        sub.xGoalInput = rossubscriber('/xGoalInput',@xGoalCallback);
        sub.yGoalInput = rossubscriber('/yGoalInput',@yGoalCallback);
        sub.tzGoalInput = rossubscriber('/tzGoalInput',@tzGoalCallrosback);
    catch
        disp(' ');
        disp('Cant communicate with goal topics!');
    end
    
    % Display
    disp(' ');
    disp('Matlab ROS node scubscribers created...');
    
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
    disp(' ');
    disp('Instantiating major structures...');
    
end

function dGps = dGpsCheck(sub)
    
    if isfieldRecursive(sub,'groundGps') == 1
        dGps = true;
        disp(' ');
        disp('Differential GPS mode activated!');
    else
        dGps = false;
        disp(' ');
        disp('Diffential GPS mode deactivated!');
    end

end

function pose = updatePose(pose,dGps)

    % Instantiate ROS callback subsciber global variables
    global wamvGpsLat wamvGpsLon wamvGpsAlt         % wamv latitude [deg], latitude [deg], and altitude [m]
    global groundGpsLat groundGpsLon groundGpsAlt   % ground latitude [deg], latitude [deg], and altitude [m]
    global imutx imuty imutz                        % euler angular position
    global imudtx imudty imudtz                     % euler angular velocity
    global imuddx imuddy imuddz                     % euler linear acceleration
    
    if dGps == 0                    % Standard GPS mode
        
        if isfieldRecursive(pose,'initialLat') == 0    % inital GPS coordinates do not exist yet
            
            % Set intial GPS
            pose.initialLat = wamvGpsLat;
            pose.initialLon = wamvGpsLon;
            pose.initialAlt = wamvGpsAlt;
                        
            % Set to local var
            x = 0; y = 0; z = 0;
        
        else                                            % initial GPS coordinates exist
            
            % Convert lattitude and longitude to meters in map
            lla = [wamvGpsLat,wamvGpsLon,wamvGpsAlt];
            llo = [pose.initialLat,pose.initialLon];
            psio = 90;
            href = -pose.initialAlt;
            posmm = lla2flat(lla, llo, psio, href);

            % Set to local var...
            x = posmm(1); y = posmm(2); z = posmm(3);
                        
        end
        
    else                                                % Differential GPS mode
        
        % Convert lattitude and longitude to meters in map
        lla = [wamvGpsLat,wamvGpsLon,wamvGpsAlt];
        llo = [groundGpsLat,groundGpsLon];
        psio = 90;
        href = -groundGpsAlt;
        posmm = lla2flat(lla, llo, psio, href);

        % Because Matlab is silly and doesn't let lla2flat output directly to matrix...
        x = posmm(1); y = posmm(2); z = posmm(3);
        
    end
                        
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

function goal = initialGoal(goal,pose)
    % Sets the initial goal of the WAM-V to the current position of the
    % WAM-V (so that it doesn't start moving when the code runs).

    % Set initial goal to current position of WAM-V
    goal.x = pose.x;
    goal.y = pose.y;
    goal.tz = pose.tz;
        
    % Display
    disp(' ');
    disp('Setting initial waypoint goal to starting position');
    
end

function goal = checkRosGoal(goal)
    % Checks the /xGoalInput and yGoalInput topics in ROS, if they are
    % different than the current value in Matlab (goal.x and goal.y), then
    % overwrite the value in Matlab

    % Instantiate ROS callback subsciber global variables
    global xGoalInput yGoalInput tzGoalInput
     
    % Check ROS goal callbacks, if different from Matlab goal, overwrite
    if goal.x ~= xGoalInput
        goal.x = xGoalInput;
    end
    if goal.y ~= yGoalInput
        goal.y = yGoalInput;
    end
    if goal.tz ~= tzGoalInput
        goal.tz = tzGoalInput;
    end
    
end

function [thrustCommands,error] = stationKeeping(goal,pose,pub,thrustCommands)

    % EVENTUALLY, WE WILL WRITE NICE FUNCTIONS FOR ALL OF THIS, BUT RUSHING
    % FOR NOW. 
    
    % Calculate errors
    error.xDist2goal = goal.x-pose.x;
    error.yDist2goal = goal.y-pose.y;
    error.head2goal = wrapToPi(goal.tz-pose.tz);
    error.dist2goal = sqrt((goal.x-pose.x)^2+(goal.y-pose.y)^2);
%     error.pathHead2goal = wrapToPi(atan2(sin(atan2(error.yDist2goal,...
%         error.xDist2goal)-error.head2goal),cos(atan2(error.yDist2goal,...
%         error.xDist2goal)-error.head2goal)));
    error.pathHead2goal = wrapToPi(atan2(sin(atan2(error.yDist2goal,...
        error.xDist2goal)),cos(atan2(error.yDist2goal,...
        error.xDist2goal))));

    if error.head2goal < 0
	error.head2goal = error.head2goal+pi;
    end
    if error.pathHead2goal < 0
	error.pathHead2goal = error.pathHead2goal+pi;
    end  
    
    % Determine behavior
    % Behavior 1 = Outside approach radius, and not facing path heading.  Pure rotation to face goal.
    % Behavior 2 = Outside approach radius, and facing path heading.  Drive forward and control path heading to goal.  
    % Behavior 3 = Inside apprach radius, and not facing path heading.  Pure rotation to face goal.
    % Behavior 4 = Inside approach radius, and facing path heading. Drive forward slowly and control path heading to goal.  
    % Behavior 5 = Inside goal radius, but not facing path heading.  Pure rotation to face goal heading.
    % Behavior 6 = Inside goal radius, and facing path heading.  Full stop.
    
    % Instantiate behavior variable
    thrustCommands.behavior = 0;
    
    % User variables
    approachRadius = 15;    % [m]
    goalRadius = 3;         % [m]
    pathArc = deg2rad(10);   % [rad]
    
    %if error.dist2goal >= approachRadius
    %    if error.pathHead2goal >= pathArc
    %        thrustCommands.behavior = 1;
    %    else
    %        thrustCommands.behavior = 2; 
    %    end
    %elseif error.dist2goal < approachRadius && error.dist2goal >= goalRadius
    %    if error.pathHead2goal >= pathArc
    %        thrustCommands.behavior = 3;
    %    else
    %        thrustCommands.behavior = 4;
    %    end
    %else
    %    if error.head2goal >= pathArc
    %        thrustCommands.behavior = 5;
    %    else
    %        thrustCommands.behavior = 6;
    %    end
    %end
    
    thrustCommands.behavior = 5;

    % PID gains
    kp.pos = 0.25;
    kp.rot = 15;
    kp.vel = 1; 
    kd.pos = 0.5;
    kd.rot = 0.3;
    kd.vel = 0.3;
    ki.pos = 0.1;
    ki.rot = 0.1;
    ki.vel = 0.1;
    
    % Controller output vector, u (all proportional PID controller for now...)
    if thrustCommands.behavior == 1
        u.Q1.prop = kp.rot*error.pathHead2goal;
        u.Q2.prop = -kp.rot*error.pathHead2goal;
        u.Q3.prop = -kp.rot*error.pathHead2goal;
        u.Q4.prop = kp.rot*error.pathHead2goal;
    elseif thrustCommands.behavior == 2
        u.Q1.prop = kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q2.prop = -kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q3.prop = -kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q4.prop = kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
    elseif thrustCommands.behavior == 3
        u.Q1.prop = kp.rot*error.pathHead2goal;
        u.Q2.prop = -kp.rot*error.pathHead2goal;
        u.Q3.prop = -kp.rot*error.pathHead2goal;
        u.Q4.prop = kp.rot*error.pathHead2goal;
    elseif thrustCommands.behavior == 4
        u.Q1.prop = kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q2.prop = -kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q3.prop = -kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
        u.Q4.prop = kp.rot*error.pathHead2goal+kp.pos*error.dist2goal;
    elseif thrustCommands.behavior == 5
        u.Q1.prop = kp.rot*error.head2goal;
        u.Q2.prop = -kp.rot*error.head2goal;
        u.Q3.prop = -kp.rot*error.head2goal;
        u.Q4.prop = kp.rot*error.head2goal;
    else
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
    
%     thrustVector = 
    
    % Publish
    Q1msg = rosmessage(pub.autoQ1);
    Q2msg = rosmessage(pub.autoQ2);
    Q3msg = rosmessage(pub.autoQ3);
    Q4msg = rosmessage(pub.autoQ4);
    
    thrustCommands.Q1
    
    Q1msg.Data = thrustCommands.Q1;
    Q2msg.Data = thrustCommands.Q2;
    Q3msg.Data = thrustCommands.Q3;
    Q4msg.Data = thrustCommands.Q4;

    send(pub.autoQ1,Q1msg);
    send(pub.autoQ2,Q2msg);
    send(pub.autoQ3,Q3msg);
    send(pub.autoQ4,Q4msg);
        
end

function displayStuff(goal,pose,thrustCommands,error)
    % Display stuff to Matlab command window, or in terminal if SSHing.
    
    % Print information
    clc;
    fprintf('Current Goal (x[m], y[m], tz[deg]:      %.1f, %.1f, %.1f\n',goal.x,goal.y,rad2deg(goal.tz));
    fprintf('Current Position (x[m], y[m], tz[deg]:  %.1f, %.1f, %.1f\n\n',pose.x,pose.y,rad2deg(pose.tz));
    fprintf('Heading error [deg]: %.1f\n',rad2deg(error.head2goal));
    fprintf('Path heading error [deg]: %.1f\n\n',rad2deg(error.pathHead2goal));
    fprintf('Thrusters [-100 to 100]:  %.1d, %.1d, %.1d, %.1d,\n\n',...
        thrustCommands.Q1,thrustCommands.Q2,thrustCommands.Q3,thrustCommands.Q4);
    fprintf('Behavior: %d\n',thrustCommands.behavior);
    
end

function out = isfieldRecursive(varargin)
%allFieldsExist = isfieldRecursive(myStructure,'fieldOfTopLevel','fieldOfFirstField','FieldOfThatNextField')
%This function has two main purposes:
%1) To check whether a field of a field of field (etc.) of a structure exists. Can be used to check as many levels deep as
%desired. Provide the first input as a structure and any number of subsequent inputs as strings
%containing field names to be checked (or cell arrays of these, see below).
%Example
% myStructure.right.directory = 'd';
%allFieldsExist = isfieldRecursive(specimen,'right','directory')
% allFieldsExist =
%      1
%2) To test multiple branches of multiple fields of structures
%recursivley until all are found to be present. To check whether more than one field exists at a
%certain level, pass a cell array of strings and fields will be searched for at that level.
%Example:
% myStructure.calibration.left.fc = 1;
% myStructure.calibration.right.fc = 1;
% myStructure.calibration.centre.fc = 1;
%allFieldsExist = isfieldRecursive(myStructure,'calibration',{'left','right','centre'})
% allFieldsExist =
%      1
%allFieldsExist = isfieldRecursive(myStructure,'calibration',{'left','right','centre'},'fc')
% allFieldsExist =
%      1
%allFieldsExist = isfieldRecursive(myStructure,'calibration',{'left','right','centre','blah'},'fc')
% allFieldsExist =
%      0
%allFieldsExist = isfieldRecursive(myStructure,'calibration',{'left','right','centre'},{'fc','kc'})
% allFieldsExist =
%      0
% myStructure.calibration.left.kc = 1;
%allFieldsExist = isfieldRecursive(myStructure,'calibration',{'left','right','centre'},{'fc','kc'})
% allFieldsExist =
%      0
% myStructure.calibration.right.kc = 1;
% myStructure.calibration.centre.kc = 1;
%allFieldsExist = isfieldRecursive(myStructure,'calibration',{'left','right','centre'},{'fc','kc'})
% allFieldsExist =
%      1
%Author: M Arthington
%Date: 2010/07/7

    out = true;
    while length(varargin)>=2 && out
        if iscell(varargin{2})
            for i=1:length(varargin{2})
                if length(varargin)>2
                    out = out && isfieldRecursive(varargin{1},varargin{2}{i},varargin{3:end});
                else
                    out = out && isfieldRecursive(varargin{1},varargin{2}{i});
                end
            end
            if out
                varargin{1} = varargin{1}.(varargin{2}{1});%Select the first cell so that the while loop can exit, even though this will already have been checked
                varargin = {varargin{1} varargin{3:end}};
            end
        else
            if out && isfield(varargin{1},varargin{2})
                varargin{1} = varargin{1}.(varargin{2});
                varargin = {varargin{1} varargin{3:end}};
            else
                out = false;
            end
        end
    end

end

function wamvGpsCallback(~,message)

    global wamvGpsLat wamvGpsLon wamvGpsAlt
    wamvGpsLat = message.Latitude;
    wamvGpsLon = message.Longitude;
    wamvGpsAlt = message.Altitude;

end

function groundGpsCallback(~,message)

    global groundGpsLat groundGpsLon groundGpsAlt
    groundGpsLat = message.Latitude;
    groundGpsLon = message.Longitude;
    groundGpsAlt = message.Altitude;

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
