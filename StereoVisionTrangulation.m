% Stereo vision validation of an IMU-based 3D human pose
% tracking system.
%
% Author: Aidan Stapleton
% Date: 05/10/2023
%

% Clear the workspace and command window
cla;
clc;
close all;
clear all;

% Get the stereo parameters for test 1A and test 1B
StereoParams = load('stereoParamsTest2_3_2.mat');
StereoParams = StereoParams.stereoParams;

% Get the original image
im1 = imread("Test3/Left/LeftImage8.png");
im2 = imread("Test3/Right/RightImage8.png");

% subplot(1, 2, 1);
% hold on;
% imshow(im1);
% impixelinfo;
% subplot(1, 2, 2);
% hold on;
% imshow(im2);
% impixelinfo;

% In this case, images are not distorted
im1_undistorted = im1;
im2_undistorted = im2;

% Convert images to greyscale for contrast enhancement
im1 = rgb2gray(im1);
im2 = rgb2gray(im2);

% Improve contrast by using adaptive histogram
ContrastedPoints1 = adapthisteq(im1);
ContrastedPoints2 = adapthisteq(im2);

Radius = 10;
Amount = 1.8;
ContrastedPoints1 = imsharpen(ContrastedPoints1, 'Radius', Radius, 'Amount', Amount);
ContrastedPoints2 = imsharpen(ContrastedPoints2, 'Radius', Radius, 'Amount', Amount);

% SURF or Harris
SURForHarris = 0;

% Harris parameters
MinQuality = 0.0001;
FilterSize = 3;

% Compute a matched set of SURF features across two images
%Points1SURF = detectSURFFeatures(ContrastedPoints1, 'ROI', [230, 260, 190, 260]);
%Points2SURF = detectSURFFeatures(ContrastedPoints2, 'ROI', [1, 1, size(im2, 2)/3, size(im2, 1)]);
ROISize = [100 100];
LeftImageJointPosition = [721, 588];
RightImageJointPosition = [644, 619];

LeftROI = [LeftImageJointPosition(1)-ROISize(1)/2, LeftImageJointPosition(2)-ROISize(2)/2, ROISize(1), ROISize(2)];
RightROI = [RightImageJointPosition(1)-ROISize(1)/2, RightImageJointPosition(2)-ROISize(2)/2, ROISize(1), ROISize(2)];

Points1Harris = detectHarrisFeatures(ContrastedPoints1, 'ROI', LeftROI, 'MinQuality', MinQuality, 'FilterSize', FilterSize);
Points2Harris = detectHarrisFeatures(ContrastedPoints2, 'ROI', RightROI, 'MinQuality', MinQuality, 'FilterSize', FilterSize);
  
%Points1 = Points1SURF;
%Points2 = Points2SURF;

Points1 = Points1Harris;
Points2 = Points2Harris;

[Descriptors1, Points1] = extractFeatures(im1, Points1);
[Descriptors2, Points2] = extractFeatures(im2, Points2);

% Calculate the corresponding pair matches based on descriptors
MatchThreshold = 40;

MatchedPairs = matchFeatures(Descriptors1, Descriptors2, 'MatchThreshold', MatchThreshold);
Points1Matched = Points1(MatchedPairs(:, 1), :);
Points2Matched = Points2(MatchedPairs(:, 2), :);

% Estimate the fundamental matrix
[F, inliersIndex, status] = estimateFundamentalMatrix(Points1Matched, Points2Matched, 'Method', 'MSAC');

% Plot the sets of inlier correspondences on the undistorted images
showMatchedFeatures(ContrastedPoints1, ContrastedPoints2, Points1Matched(inliersIndex, :), Points2Matched(inliersIndex, :), 'montage');

% Compute the 3D locations corresponding to each point
Point3D = triangulate(Points1Matched, Points2Matched, StereoParams);

% Mask the 3D points to remove outliers
Inlier3DPoints = Point3D(inliersIndex, :);

% Mask the matched points on the first image
InlierPoints1Matched = Points1Matched(inliersIndex);

% Display the inlier 3D points wrt the left camera
disp(Inlier3DPoints);




