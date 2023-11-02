%
% IMU-Based 3D Human Pose Tracking System
%
% Author: Aidan Stapleton
%
% Date: 03/10/2023
%
% Experiment 1: Elevation of the left arm from beside the
% body to above the head and back again, pausing at 8
% points of interest. The X and Z component of the left
% wrist position is tracked and compared with 
% stereo vision ground truth data.

% Clear the workspace and command window
cla;
clc;
clear;
close all;

% Read the data as floating-point numbers
Data = load('Test1Data.txt');

% Define x axis data
T = linspace(1, length(Data), length(Data));

% Calculate the timestamps for the left wrist X data
T_Mid = 170;

% Define the x-axis data for the first 8 points (T_gt_1), 
% last 8 points (T_gt_2), and all 15 points (T_gt).
T1_Offset = 10;
T_gt_1 = linspace(1, T_Mid, 8);
T_gt_1 = [T_gt_1(1), T_gt_1(2:end-2) + T1_Offset, T_gt_1(end-1)+ T1_Offset/2, T_Mid];
T_gt_2 = linspace(T_Mid, length(Data), 8);
T_gt = [T_gt_1, T_gt_2(2:end)];

% Define the measured ground truth data from point 1 to point 8
GroundTruthXPoint1To8 = -1*[0.7013, 0.707, 0.6708, 0.6337, 0.527, 0.415, 0.2807, 0.1747];
GroundTruthYPoint1To8 = [0.031, 0.0317, 0.0313, 0.0317, 0.0327, 0.0313, 0.0313, 0.0317];
GroundTruthZPoint1To8 = [0.153, 0.2847, 0.3933, 0.5487, 0.6623, 0.7323, 0.7893, 0.8063];

% Define the Xcw vector
Xcw = [0.3523, -0.3499, -1.2035];

% Define the Xc vectors
Xc1 = [0.3147, 0.2108, 1.3144];
Xc2 = [0.3102, 0.0629, 1.3546];
Xc3 = [0.2978, -0.0299, 1.3743];
Xc4 = [0.2386, -0.1667, 1.4245];
Xc5 = [0.1423, -0.2721, 1.4217];
Xc6 = [0.0452, -0.3797, 1.4225];
Xc7 = [-0.0747, -0.4134, 1.4109];
Xc8 = [-0.1836, -0.4620, 1.4081];

% The z axis in the world coordinate frame equals the inverse of the y axis
% in camera frame. X is also inverted.
IndexZ = 2;
PolarityX = -1;
PolarityZ = -1;

% Calculate the Xw vectors
XwX = [Xc1(1), Xc2(1), Xc3(1), Xc4(1), Xc5(1), Xc6(1), Xc7(1), Xc8(1)] + Xcw(1);
XwZ = [Xc1(IndexZ), Xc2(IndexZ), Xc3(IndexZ), Xc4(IndexZ), Xc5(IndexZ), Xc6(IndexZ), Xc7(IndexZ), Xc8(IndexZ)] + Xcw(IndexZ);

% Define the stereo vision ground truth data from point 1 to point 8
SVXPoint1To8 = PolarityX*XwX;
SVZPoint1To8 = PolarityZ*XwZ;

SVGroundTruthX = [SVXPoint1To8 flip(SVXPoint1To8(1:end-1))];
SVGroundTruthZ = [SVZPoint1To8 flip(SVZPoint1To8(1:end-1))];

% Define the ground truth for moving the wrist from point 1 to
% point 8 and then back down to point 1
GroundTruthX = [GroundTruthXPoint1To8 flip(GroundTruthXPoint1To8(1:end-1))];
GroundTruthY = [GroundTruthYPoint1To8 flip(GroundTruthYPoint1To8(1:end-1))];
GroundTruthZ = [GroundTruthZPoint1To8 flip(GroundTruthZPoint1To8(1:end-1))];

% Split the data into X, Y and Z
DataX = Data(:, 1);
DataY = Data(:, 2);
DataZ = Data(:, 3);

% Offset coordinate frame in the Y direction to align with
% ground truth coordinate frame
DataY = DataY + 0.03;

% Get the raw data at each ground truth time step
DataX_Aligned = DataX(round(T_gt), :);
DataZ_Aligned = DataZ(round(T_gt), :);

% Calculate the X precision for the first 7 points where
% two measurements are taken
for i = 1:7
    M1 = DataX_Aligned(i);
    M2 = DataX_Aligned(end-i+1);
    Mean = (M1 + M2)/2;
    
    PrecisionX(i) = (abs(M1-Mean) + abs(M2-Mean))/2;
end

% Calculate the Z precision for the first 7 points where
% two measurements are taken
for i = 1:7
    M1 = DataZ_Aligned(i);
    M2 = DataZ_Aligned(end-i+1);
    Mean = (M1 + M2)/2;
    
    PrecisionZ(i) = (abs(M1-Mean) + abs(M2-Mean))/2;
end

% Calculate the average precision in each axis
AvgPrecisionX = sum(PrecisionX)/7;
AvgPrecisionZ = sum(PrecisionZ)/7;

% Get the error for X and Z
ErrorX = abs(SVGroundTruthX - DataX_Aligned');
ErrorZ = abs(SVGroundTruthZ - DataZ_Aligned');

% Get the average error for X and Z
for i = 1:7
    disp(ErrorX(end-i+1));
    AvgErrorX(i) = (ErrorX(i) + ErrorX(end-i+1))/2;
    AvgErrorZ(i) = (ErrorZ(i) + ErrorZ(end-i+1))/2;
end

% Add point 8
AvgErrorX(8) = ErrorX(8);
AvgErrorZ(8) = ErrorZ(8);

% Convert to m to cm
AvgErrorX = AvgErrorX*10^2;
AvgErrorZ = AvgErrorZ*10^2;

% Calculate the average error per axis
TotalAvgErrorX = sum(AvgErrorX)/length(AvgErrorX);
TotalAvgErrorZ = sum(AvgErrorZ)/length(AvgErrorZ);

% Define colours
str = '#A8142F';
red = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;
str = '#77AC30';
green = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;
str = '#0072BD';
blue = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;

% Plot left wrist position X
figure;
hold on;
plot(T, DataX, '-o', 'Color', blue);
plot(T_gt, SVGroundTruthX, '-o', 'Color', red);
grid on;
title("Left Wrist Position X");
xlabel('Data Sample');
ylabel('Left Wrist position X (m)');
legend('Left Wrist Position X', 'Ground Truth X (Stereo Vision)');

% Plot left wrist position Z
figure;
hold on;
plot(T, DataZ, '-o', 'Color', blue);
plot(T_gt, SVGroundTruthZ, '-o', 'Color', red);
grid on;
title("Left Wrist Position Z");
xlabel('Data Sample');
ylabel('Left Wrist position Z (m)');
legend('Left Wrist Position Z', 'Ground Truth Z (Stereo Vision)');

% Plot left wrist error X
figure;
plot(T_gt, ErrorX, '-o', 'Color', red);
grid on;
title("Left Wrist X Position Error");
xlabel('Data Sample');
ylabel('Left Wrist position error X (%)');
legend('Error X (%)');

% Plot left wrist error Z
figure;
plot(T_gt, ErrorZ, '-o', 'Color', red);
grid on;
title("Left Wrist Z Position Error");
xlabel('Data Sample');
ylabel('Left Wrist position error Z (%)');
legend('Error Z (%)');

