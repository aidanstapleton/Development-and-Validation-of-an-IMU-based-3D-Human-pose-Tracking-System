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

% Validate data from the right leg moving from the initial position to
% beside of their body at 60 degrees to the horizontal and back.

% Read the data as floating-point numbers
KneeData = load('Test3ResultsKnee2.txt');
AnkleData = load('Test3ResultsAnkle2.txt');

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

% Define the erorr subsamples
KneeX1Error = abs(Xw1_Knee(1) - KneeX1SubSampled);
KneeX2Error = abs(Xw2_Knee(1) - KneeX2SubSampled);
AnkleX1Error = abs(Xw1_Ankle(1) - AnkleX1SubSampled);
AnkleX2Error = abs(Xw2_Ankle(1) - AnkleX2SubSampled);

KneeZ1Error = abs(Xw1_Knee(3) - KneeZ1SubSampled);
KneeZ2Error = abs(Xw2_Knee(3) - KneeZ2SubSampled);
AnkleZ1Error = abs(Xw1_Ankle(3) - AnkleZ1SubSampled);
AnkleZ2Error = abs(Xw2_Ankle(3) - AnkleZ2SubSampled);

% Define the average error for each point in cm
AvgKneeX1Error = (sum(KneeX1Error)/length(KneeX1Error))*10^2;
AvgKneeX2Error = (sum(KneeX2Error)/length(KneeX2Error))*10^2;
AvgAnkleX1Error = (sum(AnkleX1Error)/length(AnkleX1Error))*10^2;
AvgAnkleX2Error = (sum(AnkleX2Error)/length(AnkleX2Error))*10^2;

AvgKneeXError = (AvgKneeX1Error + AvgKneeX2Error)/2;
AvgAnkleXError = (AvgAnkleX1Error + AvgAnkleX2Error)/2;

AvgKneeZ1Error = (sum(KneeZ1Error)/length(KneeZ1Error))*10^2;
AvgKneeZ2Error = (sum(KneeZ2Error)/length(KneeZ2Error))*10^2;
AvgAnkleZ1Error = (sum(AnkleZ1Error)/length(AnkleZ1Error))*10^2;
AvgAnkleZ2Error = (sum(AnkleZ2Error)/length(AnkleZ2Error))*10^2;

AvgKneeZError = (AvgKneeZ1Error + AvgKneeZ2Error)/2;
AvgAnkleZError = (AvgAnkleZ1Error + AvgAnkleZ2Error)/2;

% --------------------------------- Accuracy X ----------------------------
% Calculate the accuracy for the X component of the knee position
KneeXAccuracyPoint1 = (1 - abs(abs(Xw1_Knee(1) - KneeX1SubSampled)./Xw1_Knee(1)))*100;
KneeXAccuracyPoint1 = max(KneeXAccuracyPoint1, 0);
AverageAccuracyKneeX1 = sum(KneeXAccuracyPoint1)/length(KneeXAccuracyPoint1);

KneeXAccuracyPoint2 = (1 - abs(abs(Xw2_Knee(1) - KneeX2SubSampled)./Xw2_Knee(1)))*100;
KneeXAccuracyPoint2 = max(KneeXAccuracyPoint2, 0);
AverageAccuracyKneeX2 = sum(KneeXAccuracyPoint2)/length(KneeXAccuracyPoint2);

% Calculate the accuracy for the X component of the ankle position
AnkleXAccuracyPoint1 = (1 - abs(abs(Xw1_Ankle(1) - AnkleX1SubSampled)./Xw1_Ankle(1)))*100;
AnkleXAccuracyPoint1 = max(AnkleXAccuracyPoint1, 0);
AverageAccuracyAnkleX1 = sum(AnkleXAccuracyPoint1)/length(AnkleXAccuracyPoint1);

AnkleXAccuracyPoint2 = (1 - abs(abs(Xw2_Ankle(1) - AnkleX2SubSampled)./Xw2_Ankle(1)))*100;
AnkleXAccuracyPoint2 = max(AnkleXAccuracyPoint2, 0);
AverageAccuracyAnkleX2 = sum(AnkleXAccuracyPoint2)/length(AnkleXAccuracyPoint2);

% --------------------------------- Accuracy Z ----------------------------
% Calculate the accuracy for the Z component of the knee position
KneeZAccuracyPoint1 = (1 - abs(abs(Xw1_Knee(3) - KneeZ1SubSampled)./Xw1_Knee(3)))*100;
KneeZAccuracyPoint1 = max(KneeZAccuracyPoint1, 0);
AverageAccuracyKneeZ1 = sum(KneeZAccuracyPoint1)/length(KneeZAccuracyPoint1);

KneeZAccuracyPoint2 = (1 - abs(abs(Xw2_Knee(3) - KneeZ2SubSampled)./Xw2_Knee(3)))*100;
KneeZAccuracyPoint2 = max(KneeZAccuracyPoint2, 0);
AverageAccuracyKneeZ2 = sum(KneeZAccuracyPoint2)/length(KneeZAccuracyPoint2);

% Calculate the accuracy for the Z component of the ankle position
AnkleZAccuracyPoint1 = (1 - abs(abs(Xw1_Ankle(3) - AnkleZ1SubSampled)./Xw1_Ankle(3)))*100;
AnkleZAccuracyPoint1 = max(AnkleZAccuracyPoint1, 0);
AverageAccuracyAnkleZ1 = sum(AnkleZAccuracyPoint1)/length(AnkleZAccuracyPoint1);

AnkleZAccuracyPoint2 = (1 - abs(abs(Xw2_Ankle(3) - AnkleZ2SubSampled)./Xw2_Ankle(3)))*100;
AnkleZAccuracyPoint2 = max(AnkleZAccuracyPoint2, 0);
AverageAccuracyAnkleZ2 = sum(AnkleZAccuracyPoint2)/length(AnkleZAccuracyPoint2);

% Display average accuracies
disp("Average Accuracy Knee X Point 1: " + num2str(AverageAccuracyKneeX1));
disp("Average Accuracy Knee X Point 2: " + num2str(AverageAccuracyKneeX2));

disp("Average Accuracy Ankle X Point 1: " + num2str(AverageAccuracyAnkleX1));
disp("Average Accuracy Ankle X Point 2: " + num2str(AverageAccuracyAnkleX2));

disp("Average Accuracy Knee Z Point 1: " + num2str(AverageAccuracyKneeZ1));
disp("Average Accuracy Knee Z Point 2: " + num2str(AverageAccuracyKneeZ2));

disp("Average Accuracy Ankle Z Point 1: " + num2str(AverageAccuracyAnkleZ1));
disp("Average Accuracy Ankle Z Point 2: " + num2str(AverageAccuracyAnkleZ2));

KneeColour = 'b';
AnkleColour = 'r';

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
% Plot X measured ground truth
% yline(KneeGroundTruthX(1), KneeColour);
% yline(KneeGroundTruthX(2), KneeColour);
% yline(AnkleGroundTruthX(1), AnkleColour);
% yline(AnkleGroundTruthX(2), AnkleColour);

% Plot X stereo vision ground truth
yline(Xw1_Knee(1), 'Color', blue, 'LABEL', 'Knee A');
yline(Xw2_Knee(1), 'Color', blue, 'LABEL', 'Knee B');
yline(Xw1_Ankle(1), 'Color', red, 'LABEL', 'Ankle A');
yline(Xw2_Ankle(1), 'Color', red, 'LABEL', 'Ankle B');

title("Right Leg Position X");
xlabel('Data Sample');
ylabel('Right Leg Position X (m)');
legend('Right Knee Position X', 'Right Ankle Position X');

figure;
hold on;
plot(T(1:450), KneeDataZ(1:450), '-o', 'Color', blue);
plot(T(1:450), AnkleDataZ(1:450), '-o', 'Color', red);
grid on;
xlim([0 500]);
ylim([-1.25 -0.6]);

% yline(KneeGroundTruthZ(1), KneeColour);
% yline(KneeGroundTruthZ(2), KneeColour);
% yline(AnkleGroundTruthZ(1), AnkleColour);
% yline(AnkleGroundTruthZ(2), AnkleColour);

% Plot Z stereo vision ground truth
yline(Xw1_Knee(3), 'Color', blue, 'LABEL', 'Knee A');
yline(Xw2_Knee(3), 'Color', blue, 'LABEL', 'Knee B');
yline(Xw1_Ankle(3), 'Color', red, 'LABEL', 'Ankle A');
yline(Xw2_Ankle(3), 'Color', red, 'LABEL', 'Ankle B');

title("Right Leg Position Z");
xlabel('Data Sample');
ylabel('Right Leg Position Z (m)');
legend('Right Knee Position Z', 'Right Ankle Position Z');
