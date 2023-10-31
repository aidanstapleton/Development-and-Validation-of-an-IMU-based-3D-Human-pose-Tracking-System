% AMME4112 - Thesis B
%
% Author: Aidan Stapleton
% Date: 012/09/2023
%

% Clear the workspace and command window
cla;
clc;
clear;
close all;

% Validate data from the right arm moving from the initial position to
% directly infront of their body with their thumb pointing left.

% Read the data as floating-point numbers
Data = load('Test1B.txt');

% Define x axis data
T = linspace(1, length(Data), length(Data));
%T_gt = linspace(1, length(Data), 10);
T_gt = [0, 63, 138, 196, 247, 301, 350, 400, 450, 500];

% Define the ground truth for moving the wrist from point 1 to
% point 8 and then back down to point 1
% For the current set of data 

% Define the Xcw vector
Xcw = [-0.3523, 0.3499, 1.2035];

% Define the camera rotation matrix
theta = 90*(pi/180);
Rx = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];

% Define the Xc vectors
Xc1 = [0.3076, 1.5600, 1.2136];
Xc2 = [-0.2322, -0.0032, 1.6429];

% Rotate Xc vectors 90 degrees about x axis
Xc1 = Xc1*Rx;
Xc2 = Xc2*Rx;
Xcw = Xcw*Rx;

% Invert Z axis
PolarityZ = -1;
Xc1(3) = Xc1(3)*PolarityZ;
Xc2(3) = Xc2(3)*PolarityZ;
Xcw(3) = Xcw(3)*PolarityZ;

% Get the Y data
DataY = Data(:, 2);

% Get each coordinate in the world coordinate frame
Xw1 = Xc1 - Xcw;
Xw2 = Xc2 - Xcw;

GroundTruthY = [0, 0.55];

% Due to depth perception difficulties, we take the x component of 
% the first point as the Y component for the second point
SVGroundTruthY = [Xw1(2), Xw2(2)];

% Calculate the errors for Y
%ErrorX = MatchedY - GroundTruthY;

% Define subsampled data
DataY1SubSampled = [-0.0058, 0.0052, 0.0113, 0.0102, 0.0003];
DataY2SubSampled = [0.5793, 0.5869, 0.5897, 0.5935, 0.5934];

% Define the erorr subsamples
ErrorY1 = abs(SVGroundTruthY(1) - DataY1SubSampled);
ErrorY2 = abs(SVGroundTruthY(2) - DataY2SubSampled);

% Define the average error for each point
AvgErrorY1 = sum(ErrorY1)/length(ErrorY1);
AvgErrorY2 = sum(ErrorY2)/length(ErrorY2);

% Convert to m to cm
AvgErrorY1 = AvgErrorY1*10^2;
AvgErrorY2 = AvgErrorY2*10^2;

% Total average error
AvgError = (AvgErrorY1 + AvgErrorY2)/2;

% Define colours
str = '#A8142F';
red = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;
str = '#77AC30';
green = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;
str = '#0072BD';
blue = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;

% Plot Y data
hold on;
plot(T(1:340), DataY(1:340), '-o', 'Color', blue);
grid on;
%yline(GroundTruthY(1), 'g');
%yline(GroundTruthY(2), 'g');
yline(SVGroundTruthY(1), 'Color', red, 'LABEL', 'Wrist A');
yline(SVGroundTruthY(2), 'Color', red, 'LABEL', 'Wrist B');
title("Left Wrist Position Y");
xlabel('Data Sample');
ylabel('Left Wrist position Y (m)');
xlim([0 400]);
ylim([-0.1 0.8]);
legend('Left Wrist Position Y', 'Ground Truth Y (Stereo Vision)');
