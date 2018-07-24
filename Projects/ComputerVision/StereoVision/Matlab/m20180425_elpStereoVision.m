function mirror_stereoscopic_vision()
    
    % Close figures, clear command window, initialize ROS
    close all;
    clc;
    rosshutdown;
    rosinit;
    
    % Initialize ROS callback subscriber
    leftCamSub = rossubscriber('/camera1/usb_cam1/image_raw/compressed',@leftCameraCallback);
    rightCamSub = rossubscriber('/camera2/usb_cam2/image_raw/compressed',@rightCameraCallback);
    
    % Declare global variables
    global leftCameraImg rightCameraImg
    
    % Pause (allow the first message to publish)
    pause(1);
    
    % Import stereoParams from matlab stereo camera calibration tool
    stereoParams = import_stereoParams();
    
    % Start looping images
    count = 1;
    while (1==1)
        
        % Convert global compressed image to local uint8
        img.L = readImage(leftCameraImg);
        img.R = readImage(rightCameraImg);
        
        % Generate disparity map
        img = generateDisparityMap(img,stereoParams);
        
        % Generate point cloud
        img = generatePointCloud(img,stereoParams);
               
        % Stitch point clouds
%         img = stitchPointCloud(img,stereoParams,count);
        
        % Plot stuff (optional)
        plotStuff(img,count);
        
        % Increment counter
        count = count+1;        
        
    end
    
end

function stereoParams = import_stereoParams()

    % Load stereoParams.mat file (must be in working directory)
    stereoParams = load('stereoParams.mat');
    stereoParams = stereoParams.stereoParams;
    
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

function img = generateDisparityMap(img,stereoParams)
        
    % Get image size information from stereoParams
    img.size = stereoParams.CameraParameters1.ImageSize;   

    % Rectify the two images
    [img.rectL,img.rectR] = rectifyStereoImages(img.L,img.R,stereoParams,...
        'OutputView','valid');

    % Create grayscale images for disparity map
    img.greyRectL = rgb2gray(img.rectL);
    img.greyRectR = rgb2gray(img.rectR);
    
    % Set disparity range (requires prior knowledge of range)
    img.dispRange = 16*7+16*7*[-1,1];             % close and long range
%     img.dispRange = 120+16*3*[-1,1];          % close range only

    % Create disparity map
    img.dispMap = disparity(img.greyRectL, img.greyRectR,...
        'Method','SemiGlobal',...                   % default SemiGlobal
        'DisparityRange',img.dispRange,...          % default [0,64]
        'BlockSize',15,...                          % default 15
        'ContrastThreshold',0.5,...                 % default 0.5
        'UniquenessThreshold',15,...                % default 15
        'DistanceThreshold',[]);                    % default []

    % Filter disparity map
    img.dispMap(img.dispMap < img.dispRange(1)) = NaN;      % explicitly remove data smaller than minimum disparity
    img.dispMap(img.dispMap > img.dispRange(2)) = NaN;      % explicitly remove data larger than maximum disparity
    img.dispMap = medfilt2(img.dispMap,[25 25]);            % 2D median filtering

end

function img = generatePointCloud(img,stereoParams)

    % Create point cloud
    points3D = reconstructScene(img.dispMap,stereoParams);
    points3D = points3D./1000;
    img.ptCloud = pointCloud(points3D,'Color',img.rectL);
    
    % Downsample point cloud
    gridSize = 0.001;
    img.ptCloud = pcdownsample(img.ptCloud,'gridAverage',gridSize);   
    
    % Run de-noise algorithm
    img.ptCloud = pcdenoise(img.ptCloud);
    img.ptCloud = removeInvalidPoints(img.ptCloud);

end

function img = stitchPointCloud(img,stereoParams,count)

    % Set reference point cloud to first cloud in sequence
    if count == 1

        % Set the first point cloud to the "reference" point cloud
        img.ptCloudRef = img.ptCloud;

    else

        % Use previous moving point cloud as reference.
        fixed = img.ptCloudRef;
        moving = img.ptCloud;

        % Apply ICP registration.
        tform = pcregrigid(moving,fixed,'Metric','pointToPlane','Extrapolate',true);
        accumTform = tform; 

        % Transform the current point cloud to the reference coordinate system defined by the first point cloud.
        accumTform = affine3d(tform.T*accumTform.T);
        ptCloudAligned = pctransform(img.ptCloud,accumTform);

        % Update the world scene.
        mergeSize = 0.01;
        if count == 2

            % Create point cloud scene relative to reference frame
            img.ptCloudScene = pcmerge(img.ptCloudRef,ptCloudAligned,mergeSize);

        else

            % Create point cloud scene relative to last frame
            img.ptCloudScene = pcmerge(img.ptCloudScene,ptCloudAligned,mergeSize);

        end

    end

end

function plotStuff(img,count)

    % Plot images, stereo anaglyph, and disparity map
    figure(1);
    subplot(2,2,1);
    imshow(img.L);
    title('Left Frame');
    subplot(2,2,3);
    imshow(img.R);
    title('Right Frame');
    subplot(2,2,2);
    imshow(stereoAnaglyph(img.rectL,img.rectR));
    title('Rectified Stereo-Anaglyph');
    subplot(2,2,4);
    imshow(img.dispMap,img.dispRange);
    title('Disparity Colormap');
    colormap(gca,jet);
    colorbar;

    % Plot the point cloud    
    zoom = 1;               % zoom coefficient (max plot forward distance in [m])
    try         
        figure(10);
        hAxes = pcshow(img.ptCloudScene,'VerticalAxis','Y','VerticalAxisDir','Down','MarkerSize',100);
        hScatter.XData = img.ptCloudScene.Location(:,1);
        hScatter.YData = img.ptCloudScene.Location(:,2);
        hScatter.ZData = img.ptCloudScene.Location(:,3);
        hScatter.CData = img.ptCloudScene.Color;    
        drawnow('limitrate');
    catch 
        figure(10);
        hAxes = pcshow(img.ptCloud,'VerticalAxis','Y','VerticalAxisDir','Down','MarkerSize',200);
        xlabel('left-right [m]');
        ylabel('down - up [m]');
        zlabel('forward [m]');
        xlim(zoom*[-2.0,3.5]);          % left - right
        ylim(zoom*[-1.5,2.0]);          % down - up
        zlim(zoom*[0,5]);               % forward
        drawnow('limitrate');
        
    end
    
end

function img = imageFromDirectory()

    % this needs modification if you're going to use this...

    %     imageDir = fullfile('snap_1.png');
    %     img.data = imageDatastore(imageDir);

    img.data = imread('snap_1.png');
    assignin('base','img',img);

end