    
function m20181114_collectStereoImgSync
% Collects, and performs rought synchronizatiion on images from two usb_cam
% devices.  The following topics must be publishing before running script:
%
%   /stereoLeft/usb_cam/camera_info
%   /stereoLeft/usb_cam/image_raw/compressed
%   /stereoRight/usb_cam/camera_info
%   /stereoRight/usb_cam/image_raw/compressed

    % Start Matlab ROS node
    close all;
    clc;
    rosshutdown;
    rosinit;
    tic;

    % Initialize loop variables
    nPairs = 100;               % total number of stereo image pairs
    snapDelay = 1;              % approximate delay in between snaps [s]
    start_count = 1;            % can change the start count in image filename
    view = 1;                   % counter for print ellipses
    width = 1280;               % image width
    height = 720;              % image height
    startCount = 1;             % starting count (for output image filename)
    syncError = 0.04;           % maximum allowable sync error [s]

    % Create subscriber with callback (interrupt) function
    stereoLeftSub = rossubscriber('/stereoLeft/usb_cam/image_raw/compressed',@stereoLeftCallback);
    stereoRightSub = rossubscriber('/stereoRight/usb_cam/image_raw/compressed',@stereoRightCallback);

    % Pause (allow first message to publish)
    pause(1);
    
    for i = startCount:startCount+nPairs-1
    
        % Collect synchronized image
        img = collectStereoPair(width,height,syncError);

        % Write images to file
        imwrite(img.L,sprintf('snapL-%d.jpeg',i));
        imwrite(img.R,sprintf('snapR-%d.jpeg',i));
        disp(['IMAGE TAKEN, PAUSING......',num2str(i)]);
    
        % Pause
        pause(snapDelay);

    end


end

function img = collectStereoPair(width,height,syncError)

    % Declare global variables
%     global stereoLeftInfo stereoLeftCompressed
%     global stereoRightInfo stereoRightCompressed
    
    global leftTime stereoLeftImg
    global rightTime stereoRightImg

    % Preallocate data matrices for speed (images must remain the same resolution or the code will break)
    img.L = zeros(height,width,3,'uint8');
    img.R = zeros(height,width,3,'uint8');
    
    % Synchronize images;
    loops = 0;
    while abs(leftTime-rightTime) > syncError
        abs(leftTime-rightTime)
        loops = loops + 1;
    end    
    img.L = readImage(stereoLeftImg);
    img.R = readImage(stereoRightImg);
    
end

function stereoLeftCallback(~,message)
    
    % Declare global variables to store position and orientation
    global stereoLeftImg leftTime

    % Set relevant message parameters to global variables
    leftTime = toc;
    stereoLeftImg = message;
    
end

function stereoRightCallback(~,message)
    
    % Declare global variables to store position and orientation
    global stereoRightImg rightTime

    % Set relevant message parameters to global variables
    rightTime = toc;
    stereoRightImg = message;
    
end