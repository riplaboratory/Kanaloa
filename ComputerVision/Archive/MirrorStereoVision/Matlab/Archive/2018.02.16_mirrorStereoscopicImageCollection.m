% Collects images from a single camera using mirrors to split the image
% Subscribes to the /usb_cam/image_raw/compressed topic

% Prep workspace
clear all;

% Start ros
rosshutdown;
rosinit;

% Initialize loop variables
n_pairs = 100;              % total number of stereo image pairs
snapDelay = 1;              % approximate delay in between snaps [s]
start_count = 1;            % can change the start count in image filename
view = 1;                   % counter for print ellipses
center_seam_frac = 0.04;    % center fraction of the image to remove due to seam (0 to 1)
width = 1280;               % image width
height = 720;               % image height

% Create subscriber with callback (interrupt) function
imgSub = rossubscriber('usb_cam/image_raw/compressed',@imageCompressedCallback);

% Declare global variables
global img_compressed

% Pause (allow first message to publish)
pause(2);

% Preallocate data matrices for speed (images must remain the same resolution or the code will break)
img = zeros(height,width,3,'uint8');                    % preallocate uint8 matrix
half_width = round((width-width*center_seam_frac)/2);   % width of the half image after excluding center seam

for i = start_count:start_count+n_pairs-1
    
    % Convert compressed image to uint8
    img = fliplr(readImage(img_compressed));
    
    % Split matrix into L and R images
    imgL = img(:,1:half_width,:);
    imgR = img(:,end-half_width+1:end,:);
    
    % Write images to file
    imwrite(img,sprintf('snap_%d.png',i));
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

function imageCompressedCallback(~,message)
    
    % Declare global variables to store position and orientation
    global img_compressed
    
    % Set relevant message parameters to global variables
    img_compressed = message;
    
end
