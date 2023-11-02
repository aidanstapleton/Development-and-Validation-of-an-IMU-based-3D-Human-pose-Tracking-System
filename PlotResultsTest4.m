%
% IMU-Based 3D Human Pose Tracking System
%
% Author: Aidan Stapleton
%
% Date: 03/10/2023
%
% Experiment 4: Elevation of the right leg beside 
% the body to a 60 degree angle of depression and back
% again. The X and Z components of the right leg 
% position is tracked and compared with stereo vision 
% ground truth data.

% Clear the workspace and command window
cla;
clc;
clear;
close all;

% Read the data as floating-point numbers
KneeData = load('Test4KneeData.txt');
AnkleData = load('Test4AnkleData.txt');

% Define x axis data
T = linspace(1, length(KneeData), length(KneeData));

% Define the ground truth for moving the wrist from point 1 to
% point 8 and then back down to point 1
KneeGroundTruthX = [0.14, 0.2157+0.14];
KneeGroundTruthZ = [-0.7763, -0.7297];
AnkleGroundTruthX = [0.14, 0.4363+0.14];
AnkleGroundTruthZ = [-1.1923, -1.0923];

% Get the X and Z data
KneeDataX = KneeData(:, 1);
KneeDataZ = KneeData(:, 3);
AnkleDataX = AnkleData(:, 1);
AnkleDataZ = AnkleData(:, 3);

% Define the Xcw vector
Xcw = [0.8153, -0.2732, 1.5035];

% Define the camera rotation matrix about the x axis
thetaX = 90.5*(pi/180);
Rx = [1 0 0; 0 cos(thetaX) -sin(thetaX); 0 sin(thetaX) cos(thetaX)];

% Define the camera rotation matrix about the y axis
thetaY = 0*(pi/180);
Ry = [cos(thetaY) 0 sin(thetaY); 0 1 0; -sin(thetaY) 0 cos(thetaY)];

% Define the camera rotation matrix about the z axis
thetaZ = 5*(pi/180);
Rz = [cos(thetaZ) -sin(thetaZ) 0; sin(thetaZ) cos(thetaZ) 0; 0 0 1];

% Define the knee and ankle coordinates in the camera frame
Xc1_Knee = [0.3834, 0.4732, 3.0474];
Xc2_Knee = [0.1942, 0.4486, 3.0804];
Xc1_Ankle = [0.3483, 0.8854, 3.5857];
Xc2_Ankle = [-0.0779, 0.7572, 3.4651];

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

% Invert X axis
PolarityX = -1;
Xc1_Knee(1) = Xc1_Knee(1)*PolarityX;
Xc2_Knee(1) = Xc2_Knee(1)*PolarityX;
Xc1_Ankle(1) = Xc1_Ankle(1)*PolarityX;
Xc2_Ankle(1) = Xc2_Ankle(1)*PolarityX;
Xcw(1) = Xcw(1)*PolarityX;

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

% Define subsampled data for the X component of the knee position
KneeX1SubSampled = [0.1692, 0.1856, 0.1549];
KneeX2SubSampled = [0.3947, 0.3774, 0.3537];

% Define subsampled data for the X component of the ankle position
AnkleX1SubSampled = [0.1670, 0.1856, 0.1612];
AnkleX2SubSampled = [0.5882, 0.5761, 0.5604];

% Define subsampled data for the Z component of the knee position
KneeZ1SubSampled = [-0.7578, -0.7603, -0.7688];
KneeZ2SubSampled = [-0.6993, -0.7100, -0.7131];

% Define subsampled data for the Z component of the ankle position
AnkleZ1SubSampled = [-1.1780, -1.1790, -1.1890];
AnkleZ2SubSampled = [-1.0690, -1.0820, -1.0880];

% -------------------------------------- Error ---------------

% Define the error for each subsubsample
KneeX1Error = abs(Xw1_Knee(1) - KneeX1SubSampled);
KneeX2Error = abs(Xw2_Knee(1) - KneeX2SubSampled);
AnkleX1Error = abs(Xw1_Ankle(1) - AnkleX1SubSampled);
AnkleX2Error = abs(Xw2_Ankle(1) - AnkleX2SubSampled);

KneeZ1Error = abs(Xw1_Knee(3) - KneeZ1SubSampled);
KneeZ2Error = abs(Xw2_Knee(3) - KneeZ2SubSampled);
AnkleZ1Error = abs(Xw1_Ankle(3) - AnkleZ1SubSampled);
AnkleZ2Error = abs(Xw2_Ankle(3) - AnkleZ2SubSampled);

% Define the average error for each point in the X axis in cm
AvgKneeX1Error = (sum(KneeX1Error)/length(KneeX1Error))*10^2;
AvgKneeX2Error = (sum(KneeX2Error)/length(KneeX2Error))*10^2;
AvgAnkleX1Error = (sum(AnkleX1Error)/length(AnkleX1Error))*10^2;
AvgAnkleX2Error = (sum(AnkleX2Error)/length(AnkleX2Error))*10^2;

% Define the average errors in the X axis for each joint
AvgKneeXError = (AvgKneeX1Error + AvgKneeX2Error)/2;
AvgAnkleXError = (AvgAnkleX1Error + AvgAnkleX2Error)/2;

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

% Plot X data
figure;
hold on;
plot(T(1:450), KneeDataX(1:450), '-o', 'Color', blue);
plot(T(1:450), AnkleDataX(1:450), '-o', 'Color', red);
xlim([0 500]);
ylim([0.1 0.7]);

% Plot the stereo vision data in the X axis as horizontals for each point
% of interest
yline(Xw1_Knee(1), 'Color', blue, 'LABEL', 'Knee A');
yline(Xw2_Knee(1), 'Color', blue, 'LABEL', 'Knee B');
yline(Xw1_Ankle(1), 'Color', red, 'LABEL', 'Ankle A');
yline(Xw2_Ankle(1), 'Color', red, 'LABEL', 'Ankle B');

title("Right Leg Position X");
xlabel('Data Sample');
ylabel('Right Leg Position X (m)');
legend('Right Knee Position X', 'Right Ankle Position X');

% Plot Z data
figure;
hold on;
plot(T(1:450), KneeDataZ(1:450), '-o', 'Color', blue);
plot(T(1:450), AnkleDataZ(1:450), '-o', 'Color', red);
grid on;
xlim([0 500]);
ylim([-1.25 -0.6]);

% Plot the stereo vision data in the Z axis as horizontals for each point
% of interest
yline(Xw1_Knee(3), 'Color', blue, 'LABEL', 'Knee A');
yline(Xw2_Knee(3), 'Color', blue, 'LABEL', 'Knee B');
yline(Xw1_Ankle(3), 'Color', red, 'LABEL', 'Ankle A');
yline(Xw2_Ankle(3), 'Color', red, 'LABEL', 'Ankle B');

title("Right Leg Position Z");
xlabel('Data Sample');
ylabel('Right Leg Position Z (m)');
legend('Right Knee Position Z', 'Right Ankle Position Z');
