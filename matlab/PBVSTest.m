%% This script is used to test IBVS
% @Victor You will need the QR Code toolbox: go to HOME -> Add-Ons -> Get
% Add-Ons -> search for "qr code" -> Install "QR Code reader example"

%% Step 1: Initialise the robot
clear all; close all;
clc;
ppr = PingPongRobot();
ppr.PlotAndColourRobot();
hold on;

rosMasterURI = 'http://192.168.1.118:11311'; % default Ubuntu PC local IP
rosIP = '192.168.1.116'; % default Windows PC (MATLAB) local IP
rosRW = ROSRobotWrapper(ppr, rosMasterURI, rosIP);

%% Start updating origin pose and joint angles
rosRW.startRobotUpdate();

%% E-Stop
rosRW.eStopRobot(true);

%% Turn off E-Stop
rosRW.eStopRobot(false);

%% TESTING JOGGING
rosRW.jogRobot([-0.05,0.2,-0.2,0,0,0,0]);

%% Stop updating origin pose and joint angles
rosRW.stopRobotUpdate();

%% Shutdown ROS
rosshutdown;

%% PBVS ROS TEST
try delete(pbvsRW); end;
pbvsRW = LivePBVSWrapper(rosRW);
pbvsRW.init(0.03, [480 360]);
pbvsRW.enableROSUpdate(true);

%%
pbvsRW.startPBVS();
%%
pbvsRW.stopPBVS();
%% manual
pbvsRW.robotPBVSControl();

%% while loop test
while(1)
    rosRW.updateRobot();
    pbvsRW.robotPBVSControl();
end
%% QR square size (from centers)
try delete(cam); end;
close all; clc;
qrSize = 0.1; % 10cm

realC = [-0.5*qrSize; 0.5*qrSize; 0];
realX = [0.5*qrSize; 0.5*qrSize; 0];
realY = [-0.5*qrSize; -0.5*qrSize; 0];

qrReal = [realC realX realY realC]

cam = webcam('Logitech Webcam C930e');
cam.Resolution = '640x480';
test_cam = CentralCamera('name', 'test_cam', 'focal', 0.05, 'pixel', 10e-5, 'resolution', [640 480], ...
                        'pose', troty(deg2rad(pi/2)));
figure(1);
figure(2);
xlim([-0.2 0.2]);
ylim([-0.2 0.2]);
zlim([-0.2 1.0]);
test_cam.plot_camera();
axis equal;

while(1)
    img = snapshot(cam);
%     img = imread('test_img1.jpg');
    bw = imbinarize(rgb2gray(img));
    [Centroid, bw2, flag, width, bbox] = detectFinder(bw);
    markers = Centroid(logical(flag),:);
    [points ~] = size(markers);
    if points > 2
        [idxC, idxX, idxY] = finderPos(markers);
        qrUV = [markers(idxC, :)' markers(idxX, :)' markers(idxY, :)' markers(idxC, :)']
        tr_point = test_cam.estpose(qrReal, qrUV);
        figure(2);
        hold on;
        try delete(tr_h); end
        tr_h = trplot(tr_point, 'length', 0.1);
        
        imgf = img;
        imgf = insertShape(imgf, 'FilledCircle', [markers(idxC, :) 10], 'Color', 'red');
        imgf = insertShape(imgf, 'FilledCircle', [markers(idxX, :) 10], 'Color', 'blue');
        imgf = insertShape(imgf, 'FilledCircle', [markers(idxY, :) 10], 'Color', 'yellow');
        figure(1);
        imshow(imgf)
    else
        figure(1);
        imshow(img)
    end
end
%%
test_cam.plot_camera();
hold on; axis equal;
trplot(test_tr);

%%
close all; clc;
try delete(cam); end
cam = webcam('Logitech Webcam C930e');
cam.Resolution = '1280x720';
test_cam = CentralCamera('name', 'test_cam', 'focal', 0.0513, 'pixel', 10e-5, 'resolution', [1280 720]);

figure(1);
figure(2);
xlim([-0.2 0.2]);
zlim([-0.2 0.2]);
ylim([-0.2 1.0]);
camT = trotx(pi/2)*trotz(pi)*troty(pi);
test_cam.plot_camera('Tcam', camT);
axis equal;

original = imread('retreat_code.png');
original = rgb2gray(original);
ptsOriginal  = detectSURFFeatures(original);
[featuresOriginal,validPtsOriginal] = extractFeatures(original,ptsOriginal);
[width, height,~] = size(original);
corners = [0,0;height,0;height,width;0,width];

qrSize = 0.2;
realCorner1 = [0.5*qrSize; -0.5*qrSize; 0];
realCorner2 = [-0.5*qrSize; -0.5*qrSize; 0];
realCorner3 = [-0.5*qrSize; 0.5*qrSize; 0];
realCorner4 = [0.5*qrSize; 0.5*qrSize; 0];
qrReal = [realCorner1 realCorner2 realCorner3 realCorner4]

while(1)
    tic
    distorted = cam.snapshot();
    distorted = rgb2gray(distorted);
    ptsDistorted = detectSURFFeatures(distorted);
    [featuresDistorted,validPtsDistorted] = extractFeatures(distorted,ptsDistorted);
    index_pairs = matchFeatures(featuresOriginal,featuresDistorted);
    [points ~] = size(index_pairs);

    matchedPtsOriginal  = validPtsOriginal(index_pairs(:,1));
    matchedPtsDistorted = validPtsDistorted(index_pairs(:,2));

    % figure; 
    % showMatchedFeatures(original,distorted,...
    %     matchedPtsOriginal,matchedPtsDistorted);
    % title('Matched SURF points,including outliers');

    try
        [tform,inlierPtsDistorted,inlierPtsOriginal] = ...
            estimateGeometricTransform(matchedPtsDistorted,matchedPtsOriginal, 'projective');
    catch
        continue;
    end

    % figure;
    % showMatchedFeatures(original,distorted,...
    %     inlierPtsOriginal,inlierPtsDistorted);
    % title('Matched inlier points');

    [newcorners(:,1) newcorners(:,2)] = transformPointsInverse(tform, corners(:,1),corners(:,2));
    tr_point = test_cam.estpose(qrReal, newcorners');
    tr_point = inv(tr_point*inv(camT));
%     [cam_world_ori, cam_world_pos] = estimateWorldCameraPose(double(newcorners),qrReal',cameraParams);
%     tr_cam_point = eye(4); tr_cam_point(1:3,1:3) = cam_world_ori; tr_cam_point(1:3,4) = cam_world_pos;
%     tr_point = inv(tr_cam_point*inv(camT));
    toc
    
    figure(1);
    imshow(distorted); hold on;
    patch(newcorners(:,1),newcorners(:,2),[0 1 0],'FaceAlpha',0.5);
    
    figure(2);
    hold on;
    try delete(tr_h); end
    tr_h = trplot(tr_point, 'length', 0.1);
end
