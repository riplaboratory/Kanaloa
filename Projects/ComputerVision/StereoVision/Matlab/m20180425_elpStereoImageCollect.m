% Collects images from a ELP USB Stereo Camera

% Prep workspace
clear all;

% Start ros
rosshutdown;
rosinit;

% Initialize loop variables
n_pairs = 150;              % total number of stereo image pairs
snapDelay = 1;              % approximate delay in between snaps [s]
start_count = 1;            % can change the start count in image filename
view = 1;                   % counter for print ellipses
width = 1280;               % image width
height = 720;               % image height

% Create subscriber with callback (interrupt) function
leftCamSub = rossubscriber('/camera1/usb_cam1/image_raw/compressed',@leftCameraCallback);
rightCamSub = rossubscriber('/camera2/usb_cam2/image_raw/compressed',@rightCameraCallback);

% Declare global variables
global leftCameraImg rightCameraImg

% Pause (allow first message to publish)
pause(2);

% Preallocate data matrices for speed (images must remain the same resolution or the code will break)
imgL = zeros(height,width,3,'uint8');
imgR = zeros(height,width,3,'uint8');

for i = start_count:start_count+n_pairs-1
    
    % Convert compressed image to uint8
    imgL = readImage(leftCameraImg);
    imgR = readImage(rightCameraImg);
        
    % Write images to file
    imwrite(imgL,sprintf('snap_L_%d.png',i));
    imwrite(imgR,sprintf('snap_R_%d.png',i));
    
    if view == 1
        disp(['IMAGE TAKEN, PAUSING......',num2str(i)]);
        view = 2;
    else
        disp(['IMAGE TAKEN, PAUSING ',num2str(i)]);
        view = 1;
    end
    
    pause(snapDelay);

    
end

function leftCameraCallback(~,message)
    
    % Declare global variables to store position and orientation
    global leftCameraImg
    
    % Set relevant message parameters to global variables
    leftCameraImg = message;
    
end

function rightCameraCallback(~,message)
    
    % Declare global variables to store position and orientation
    global rightCameraImg
    
    % Set relevant message parameters to global variables
    rightCameraImg = message;
    
end
