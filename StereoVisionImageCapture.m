%
% IMU-Based 3D Human Pose Tracking System
%
% Author: Aidan Stapleton
%
% Date: 03/10/2023
%
% Connect to the pair of Microsoft LifeCam HD-3000s.
%
% Calibration:      CameraMode = 0
%
% Tracking:         CameraMode = 1
%
% Camera Preview:   CameraMode = 2

% Clear the workspace and command window
cla;
clc;
clear;
close all;

% Settings
NumCalibPairs = 15;
CameraMode = 0;         

% Get an array containing all connected cameras
cam_List = webcamlist; 

% Access the right and left camera
LeftCam  = webcam(2);      
RightCam  = webcam(1);    

% If calibration is required
switch CameraMode
    
    % Calibration
    case 0
        
        % Capture new calibration images
        for CalibPair = 1:NumCalibPairs

            % Get the next image pair
            LeftImage = snapshot(LeftCam);
            RightImage = snapshot(RightCam);

            % Define the next image pair names
            LeftImageName = strcat('LeftImage', num2str(CalibPair), '.png');
            RightImageName = strcat('RightImage', num2str(CalibPair), '.png');

            % Write each image to the calibration file
            imwrite(LeftImage, strcat('Calibration/Left/', LeftImageName), 'PNG');
            imwrite(RightImage, strcat('Calibration/Right/', RightImageName), 'PNG');
        end
        
    % Tracking    
    case 1
    
        % Define the number of image pairs
        NumTrackingPairs = 40;

        % Capture new tracking images
        for TrackingPair = 1:NumTrackingPairs

            % Get the next image pair
            LeftImage = snapshot(LeftCam);
            RightImage = snapshot(RightCam);

            % Define the next image pair names
            LeftImageName = strcat('LeftImage', num2str(TrackingPair), '.png');
            RightImageName = strcat('RightImage', num2str(TrackingPair), '.png');

            % Write each image to the tracking file
            imwrite(LeftImage, strcat('Test3/Left/', LeftImageName), 'PNG');
            imwrite(RightImage, strcat('Test3/Right/', RightImageName), 'PNG');
        end
        
    case 2
        
        % Preview both cameras
        preview(LeftCam);
        preview(RightCam);
end


