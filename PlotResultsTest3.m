%
% IMU-Based 3D Human Pose Tracking System
%
% Author: Aidan Stapleton
%
% Date: 03/10/2023
%
% Experiment 3: Elevation of the right leg in front of 
% the body to a 30 degree angle of depression and back
% again. The Y and Z components of the right leg 
% position is tracked and compared with stereo vision 
% ground truth data.

% Clear the workspace and command window
cla;
clc;
clear;
close all;

% Read the data as floating-point numbers
KneeData = load('Test3KneeData.txt');
AnkleData = load('Test3AnkleData.txt');

% Define x axis data
T = linspace(1, length(KneeData), length(KneeData));

% Get the Y and Z data
KneeDataY = KneeData(:, 2);
KneeDataZ = KneeData(:, 3);
AnkleDataY = AnkleData(:, 2);
AnkleDataZ = AnkleData(:, 3);

% Define the Xcw vector
Xcw = [0.8153, -0.2732, 1.5035];

% Define the camera rotation matrix about the x axis
thetaX = 90*(pi/180);
Rx = [1 0 0; 0 cos(thetaX) -sin(thetaX); 0 sin(thetaX) cos(thetaX)];

% Define the camera rotation matrix about the y axis
thetaY = 0*(pi/180);
Ry = [cos(thetaY) 0 sin(thetaY); 0 1 0; -sin(thetaY) 0 cos(thetaY)];

% Define the camera rotation matrix about the z axis
thetaZ = 0*(pi/180);
Rz = [cos(thetaZ) -sin(thetaZ) 0; sin(thetaZ) cos(thetaZ) 0; 0 0 1];

% Define the stereo vision knee and ankle positions 
% in the camera coordinate frame 
Xc1_Knee = [0.8125, 0.4685, 3.1336];
Xc2_Knee = [0.4678, 0.2110, 2.9218];
Xc1_Ankle = [0.7896, 0.8570, 3.5043];
Xc2_Ankle = [0.0951, 0.3959, 3.2183];

% Rotate and invert camera axes to roughly align with the world
% coordinate frame.
% Apply a thetaX degree rotation about x
Xc1_Knee = Xc1_Knee*Rx;
Xc2_Knee = Xc2_Knee*Rx;
Xc1_Ankle = Xc1_Ankle*Rx;
Xc2_Ankle = Xc2_Ankle*Rx;

% Apply a thetaY degree rotation about y
Xc1_Knee = Xc1_Knee*Ry;
Xc2_Knee = Xc2_Knee*Ry;
Xc1_Ankle = Xc1_Ankle*Ry;
Xc2_Ankle = Xc2_Ankle*Ry;

% Apply a thetaZ degree rotation about z
Xc1_Knee = Xc1_Knee*Rz;
Xc2_Knee = Xc2_Knee*Rz;
Xc1_Ankle = Xc1_Ankle*Rz;
Xc2_Ankle = Xc2_Ankle*Rz;

% Invert Z axis
PolarityZ = -1;
Xc1_Knee(2) = Xc1_Knee(2)*PolarityZ;
Xc2_Knee(2) = Xc2_Knee(2)*PolarityZ;
Xc1_Ankle(2) = Xc1_Ankle(2)*PolarityZ;
Xc2_Ankle(2) = Xc2_Ankle(2)*PolarityZ;
Xcw(3) = Xcw(2)*PolarityZ;

% Get the knee and ankle coordinates in the world coordinate frame
Xw1_Knee = Xc1_Knee - Xcw;
Xw2_Knee = Xc2_Knee - Xcw;
Xw1_Ankle = Xc1_Ankle - Xcw;
Xw2_Ankle = Xc2_Ankle - Xcw;

% Invert X axis
PolarityX = -1;
Xw1_Knee(1) = Xw1_Knee(1)*PolarityX;
Xw2_Knee(1) = Xw2_Knee(1)*PolarityX;
Xw1_Ankle(1) = Xw1_Ankle(1)*PolarityX;
Xw2_Ankle(1) = Xw2_Ankle(1)*PolarityX;

% Define subsampled data for the Y component of the knee position
KneeY1SubSampled = [0.0006, 0.0072, 0.0014];
KneeY2SubSampled = [0.4022, 0.3962, 0.3901];

% Define subsampled data for the Y component of the ankle position
AnkleY1SubSampled = [0.0015, 0.0022, 0.0031];
AnkleY2SubSampled = [0.7673, 0.7661, 0.7410];

% Define subsampled data for the Z component of the knee position
KneeZ1SubSampled = [-0.7747, -0.7696, -0.7682];
KneeZ2SubSampled = [-0.5009, -0.4923, -0.5028];

% Define subsampled data for the Z component of the ankle position
AnkleZ1SubSampled = [-1.1940, -1.1890, -1.1880];
AnkleZ2SubSampled = [-0.7390, -0.7179, -0.7435];

% -------------------------------------- Error ----------------------------

% Define the error for each subsubsample
KneeY1Error = abs(Xw1_Knee(1) - KneeY1SubSampled);
KneeY2Error = abs(Xw2_Knee(1) - KneeY2SubSampled);
AnkleY1Error = abs(Xw1_Ankle(1) - AnkleY1SubSampled);
AnkleY2Error = abs(Xw2_Ankle(1) - AnkleY2SubSampled);

KneeZ1Error = abs(Xw1_Knee(3) - KneeZ1SubSampled);
KneeZ2Error = abs(Xw2_Knee(3) - KneeZ2SubSampled);
AnkleZ1Error = abs(Xw1_Ankle(3) - AnkleZ1SubSampled);
AnkleZ2Error = abs(Xw2_Ankle(3) - AnkleZ2SubSampled);

% Define the average error for each point in the Y axis in cm
AvgKneeY1Error = (sum(KneeY1Error)/length(KneeY1Error))*10^2;
AvgKneeY2Error = (sum(KneeY2Error)/length(KneeY2Error))*10^2;
AvgAnkleY1Error = (sum(AnkleY1Error)/length(AnkleY1Error))*10^2;
AvgAnkleY2Error = (sum(AnkleY2Error)/length(AnkleY2Error))*10^2;

% Define the average errors in the Y axis for each joint
AvgKneeYError = (AvgKneeY1Error + AvgKneeY2Error)/2;
AvgAnkleYError = (AvgAnkleY1Error + AvgAnkleY2Error)/2;

% Define the average error for each point in the Z axis in cm
AvgKneeZ1Error = (sum(KneeZ1Error)/length(KneeZ1Error))*10^2;
AvgKneeZ2Error = (sum(KneeZ2Error)/length(KneeZ2Error))*10^2;
AvgAnkleZ1Error = (sum(AnkleZ1Error)/length(AnkleZ1Error))*10^2;
AvgAnkleZ2Error = (sum(AnkleZ2Error)/length(AnkleZ2Error))*10^2;

% Define the average errors in the Z axis for each joint
AvgKneeZError = (AvgKneeZ1Error + AvgKneeZ2Error)/2;
AvgAnkleZError = (AvgAnkleZ1Error + AvgAnkleZ2Error)/2;

% Define colours
str = '#A8142F';
red = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;
str = '#77AC30';
green = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;
str = '#0072BD';
blue = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;

% Plot Y data
figure;
hold on;
plot(T(1:350), KneeDataY(1:350), '-o', 'Color', blue);
plot(T(1:350), AnkleDataY(1:350), '-o', 'Color', red);
grid on;
xlim([0 400]);
ylim([-0.1 0.9]);

% Plot the stereo vision data in the Z axis as horizontals for each point
% of interest

% Invert the x value as we are measuring y in the x
% direction along the wall
yline(Xw1_Knee(1), 'Color', blue, 'LABEL', 'Knee A', 'LabelHorizontalAlignment', 'right');
yline(Xw2_Knee(1), 'Color', blue, 'LABEL', 'Knee B', 'LabelHorizontalAlignment', 'right');
yline(Xw1_Ankle(1), 'Color', red, 'LABEL', 'Ankle A', 'LabelHorizontalAlignment', 'right');
yline(Xw2_Ankle(1), 'Color', red, 'LABEL', 'Ankle B', 'LabelHorizontalAlignment', 'right');

title("Right Leg Position Y");
xlabel('Data Sample');
ylabel('Right Leg Position Y (m)');
legend('Right Knee Position Y', 'Right Ankle Position Y');

% Plot Z data
figure;
hold on;
plot(T(1:350), KneeDataZ(1:350), '-o', 'Color', blue);
plot(T(1:350), AnkleDataZ(1:350), '-o','Color', red);
grid on;
xlim([0 400]);
ylim([-1.3 -0.3]);

% Plot the stereo vision data in the Z axis as horizontals for each point
% of interest
yline(Xw1_Knee(3), 'Color', blue, 'LABEL', 'Knee A', 'LabelHorizontalAlignment', 'right');
yline(Xw2_Knee(3), 'Color', blue, 'LABEL', 'Knee B', 'LabelHorizontalAlignment', 'right');
yline(Xw1_Ankle(3), 'Color', red, 'LABEL', 'Ankle A', 'LabelHorizontalAlignment', 'right');
yline(Xw2_Ankle(3), 'Color', red, 'LABEL', 'Ankle B', 'LabelHorizontalAlignment', 'right');

title("Right Leg Position Z");
xlabel('Data Sample');
ylabel('Right Leg Position Z (m)');
legend('Right Knee Position Z', 'Right Ankle Position Z');
