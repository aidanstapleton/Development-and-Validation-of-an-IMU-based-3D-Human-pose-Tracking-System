# This script reads human pose data from UDP wifi communication protocol and displays 
# the data as a kinematic model in real time.

# Author: Aidan Stapleton

# Import packages
import socket
import re
import matplotlib.pyplot as plt
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion

# IMU tracking index
Index = 1
InitialiseIMUCount = 0
IMUsInUse = 9

#localIP     = "192.168.0.127"           # Laptop IP address
localIP     = "192.168.0.32"           # Desktop IP address
localPort   = 2390
bufferSize  = 1024
FirstEpochFlag = 1

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))

print("UDP server up and listening")

# Generate an updatable figure
plt.ion()
fig = plt.figure(figsize=(8, 8))
fig.canvas.manager.set_window_title("IMU-based Human Pose Tracking System")
ax = fig.add_subplot(111, projection='3d')

# Define axes limits
AxesLimit = 1.5
ax.axes.set_xlim3d(left=-AxesLimit, right=AxesLimit)
ax.axes.set_ylim3d(bottom=-AxesLimit, top=AxesLimit)
ax.axes.set_zlim3d(bottom=-AxesLimit, top=AxesLimit)
ax.set_xlabel("X axis (m)")
ax.set_ylabel("Y axis (m)")
ax.set_zlabel("Z axis (m)")

ModelColour = "red"

HeadPlot, = ax.plot3D([], [], [], ModelColour)
UpperTorsoPlot, = ax.plot3D([], [], [], ModelColour, dash_capstyle='round')
LowerTorsoPlot, = ax.plot3D([], [], [], ModelColour, dash_capstyle='round')
RightShoulderPlot, = ax.plot3D([], [], [], ModelColour)
RightArmPlot, = ax.plot3D([], [], [], ModelColour)
RightForearmPlot, = ax.plot3D([], [], [], ModelColour)
LeftShoulderPlot, = ax.plot3D([], [], [], ModelColour)
LeftArmPlot, = ax.plot3D([], [], [], ModelColour)
LeftForearmPlot, = ax.plot3D([], [], [], ModelColour)
 
RightHipPlot, = ax.plot3D([], [], [], ModelColour)
RightThighPlot, = ax.plot3D([], [], [], ModelColour)
RightLegPlot, = ax.plot3D([], [], [], ModelColour)
LeftHipPlot, = ax.plot3D([], [], [], ModelColour)
LeftThighPlot, = ax.plot3D([], [], [], ModelColour)
LeftLegPlot, = ax.plot3D([], [], [], ModelColour)

# Create and open the file with the intention to read and write
#KneeResults = open("StereoVisionData/Test2ResultsKnee4.txt", "w+")
#AnkleResults = open("StereoVisionData/Test2ResultsAnkle4.txt", "w+")

def KinematicModelSetup():

    # Declare the model origin
    global Origin

    # Declare limb lengths, position variables and body part vectors --------------------------------------------
    global UpperTorsoLength, LowerTorsoLength, UpperArmLength, ForeArmLength, ThighLength, LegLength, ShoulderWidth, WaistWidth
    global HeadInitialPosition, NeckInitialPosition, WaistInitialPosition, RightShoulderInitialPosition, LeftShoulderInitialPosition, \
    RightElbowInitialPosition, LeftElbowInitialPosition, RightWristInitialPosition, LeftWristInitialPosition, \
    RightHipInitialPosition, LeftHipInitialPosition, RightKneeInitialPosition, LeftKneeInitialPosition, \
    RightAnkleInitialPosition, LeftAnkleInitialPosition, ModelYInitialPosition
    global HeadInitialPose, UpperTorsoInitialPose, LowerTorsoInitialPose, RightShoulderInitialPose, RightUpperArmInitialPose, \
    LeftShoulderInitialPose, LeftUpperArmInitialPose, RightForearmInitialPose, LeftForearmInitialPose, RightHipInitialPose, \
    LeftHipInitialPose, RightThighInitialPose, LeftThighInitialPose, RightLegInitialPose, LeftLegInitialPose

    # Define body dimeansions -----------------------------------------------------------------------------------
    HeadHeight = 0.25
    ChestHeight = 0.15
    UpperTorsoLength = 0.275
    LowerTorsoLength = 0.275
    UpperArmLength = 0.28
    ForeArmLength = 0.265
    ThighLength = 0.5
    LegLength = 0.42
    ShoulderWidth = 0.31
    WaistWidth = 0.28
    #LegAngleAtShoulderWidth = 15*(math.pi/180)

    # Deine the model origin
    Origin = [0, 0, 0]

    # Define the initial Y position for the model
    ModelYInitialPosition = 0

    # Define initial joint positions ----------------------------------------------------------------------------

    # Head
    HeadInitialPosition = [0, ModelYInitialPosition, UpperTorsoLength + HeadHeight]

    # Torso
    NeckInitialPosition = [0, ModelYInitialPosition, UpperTorsoLength]
    WaistInitialPosition = [0, ModelYInitialPosition, -1*(LowerTorsoLength)]

    # Right Arm
    RightShoulderInitialPosition = [ShoulderWidth/2, ModelYInitialPosition, UpperTorsoLength]
    RightElbowInitialPosition = [ShoulderWidth/2 + UpperArmLength, ModelYInitialPosition, ChestHeight + (UpperTorsoLength-ChestHeight)/2]
    RightWristInitialPosition = [ShoulderWidth/2 + UpperArmLength + ForeArmLength, ModelYInitialPosition, ChestHeight]

    # Left Arm
    LeftShoulderInitialPosition = [-1*(ShoulderWidth/2), ModelYInitialPosition, UpperTorsoLength]
    LeftElbowInitialPosition = [-1*(ShoulderWidth/2 + UpperArmLength), ModelYInitialPosition, ChestHeight + (UpperTorsoLength-ChestHeight)/2]
    LeftWristInitialPosition = [-1*(ShoulderWidth/2 + UpperArmLength+ForeArmLength), ModelYInitialPosition, ChestHeight]

    # Right Leg
    RightHipInitialPosition = [WaistWidth/2, ModelYInitialPosition, WaistInitialPosition[2]]
    RightKneeInitialPosition = [WaistWidth/2, ModelYInitialPosition, WaistInitialPosition[2] - 1*(ThighLength)]
    RightAnkleInitialPosition = [WaistWidth/2, ModelYInitialPosition, WaistInitialPosition[2] - 1*(ThighLength + LegLength)]

    # Left Leg
    LeftHipInitialPosition = [-1*(WaistWidth/2), ModelYInitialPosition, WaistInitialPosition[2]]
    LeftKneeInitialPosition = [-1*(WaistWidth/2), ModelYInitialPosition, WaistInitialPosition[2] - 1*(ThighLength)]
    LeftAnkleInitialPosition = [-1*(WaistWidth/2), ModelYInitialPosition, WaistInitialPosition[2] - 1*(ThighLength + LegLength)]
    
    # Define initial body part vectors ----------------------------------------------------------------------------------
    HeadInitialPose = np.subtract(HeadInitialPosition, NeckInitialPosition)
    UpperTorsoInitialPose = np.subtract(NeckInitialPosition, Origin)
    LowerTorsoInitialPose = np.subtract(WaistInitialPosition, Origin)
    RightShoulderInitialPose = np.subtract(RightShoulderInitialPosition, NeckInitialPosition)
    RightUpperArmInitialPose = np.subtract(RightElbowInitialPosition, RightShoulderInitialPosition)
    RightForearmInitialPose = np.subtract(RightWristInitialPosition, RightElbowInitialPosition)
    LeftShoulderInitialPose = np.subtract(LeftShoulderInitialPosition, NeckInitialPosition)
    LeftUpperArmInitialPose = np.subtract(LeftElbowInitialPosition, LeftShoulderInitialPosition)
    LeftForearmInitialPose = np.subtract(LeftWristInitialPosition, LeftElbowInitialPosition)
    
    RightHipInitialPose = np.subtract(RightHipInitialPosition, WaistInitialPosition)
    RightThighInitialPose = np.subtract(RightKneeInitialPosition, RightHipInitialPosition)
    RightLegInitialPose = np.subtract(RightAnkleInitialPosition, RightKneeInitialPosition)
    LeftHipInitialPose = np.subtract(LeftHipInitialPosition, WaistInitialPosition)
    LeftThighInitialPose = np.subtract(LeftKneeInitialPosition, LeftHipInitialPosition)
    LeftLegInitialPose = np.subtract(LeftAnkleInitialPosition, LeftKneeInitialPosition)
    
def SetInitialPose():
    
    # Setup the initial kinematic model pose
    HeadPlot.set_data([NeckInitialPosition[0], HeadInitialPosition[0]], [NeckInitialPosition[1], HeadInitialPosition[1]])
    HeadPlot.set_3d_properties([NeckInitialPosition[2], HeadInitialPosition[2]])

    UpperTorsoPlot.set_data([NeckInitialPosition[0], Origin[0]], [NeckInitialPosition[1], Origin[1]])
    UpperTorsoPlot.set_3d_properties([NeckInitialPosition[2], Origin[2]])

    LowerTorsoPlot.set_data([WaistInitialPosition[0], Origin[0]], [WaistInitialPosition[1], Origin[1]])
    LowerTorsoPlot.set_3d_properties([WaistInitialPosition[2], Origin[2]])

    RightShoulderPlot.set_data([RightShoulderInitialPosition[0], NeckInitialPosition[0]], [RightShoulderInitialPosition[1], NeckInitialPosition[1]])
    RightShoulderPlot.set_3d_properties([RightShoulderInitialPosition[2], NeckInitialPosition[2]])

    RightArmPlot.set_data([RightElbowInitialPosition[0], RightShoulderInitialPosition[0]], [RightElbowInitialPosition[1], RightShoulderInitialPosition[1]])
    RightArmPlot.set_3d_properties([RightElbowInitialPosition[2], RightShoulderInitialPosition[2]])
    
    RightForearmPlot.set_data([RightWristInitialPosition[0], RightElbowInitialPosition[0]], [RightWristInitialPosition[1], RightElbowInitialPosition[1]])
    RightForearmPlot.set_3d_properties([RightWristInitialPosition[2], RightElbowInitialPosition[2]])

    LeftShoulderPlot.set_data([LeftShoulderInitialPosition[0], NeckInitialPosition[0]], [LeftShoulderInitialPosition[1], NeckInitialPosition[1]])
    LeftShoulderPlot.set_3d_properties([LeftShoulderInitialPosition[2], NeckInitialPosition[2]])

    LeftArmPlot.set_data([LeftElbowInitialPosition[0], LeftShoulderInitialPosition[0]], [LeftElbowInitialPosition[1], LeftShoulderInitialPosition[1]])
    LeftArmPlot.set_3d_properties([LeftElbowInitialPosition[2], LeftShoulderInitialPosition[2]])
    
    LeftForearmPlot.set_data([LeftWristInitialPosition[0], LeftElbowInitialPosition[0]], [LeftWristInitialPosition[1], LeftElbowInitialPosition[1]])
    LeftForearmPlot.set_3d_properties([LeftWristInitialPosition[2], LeftElbowInitialPosition[2]])

    RightHipPlot.set_data([WaistInitialPosition[0], RightHipInitialPosition[0]], [WaistInitialPosition[1], RightHipInitialPosition[1]])
    RightHipPlot.set_3d_properties([WaistInitialPosition[2], RightHipInitialPosition[2]])

    RightThighPlot.set_data([RightHipInitialPosition[0], RightKneeInitialPosition[0]], [RightHipInitialPosition[1], RightKneeInitialPosition[1]])
    RightThighPlot.set_3d_properties([RightHipInitialPosition[2], RightKneeInitialPosition[2]])

    RightLegPlot.set_data([RightKneeInitialPosition[0], RightAnkleInitialPosition[0]], [RightKneeInitialPosition[1], RightAnkleInitialPosition[1]])
    RightLegPlot.set_3d_properties([RightKneeInitialPosition[2], RightAnkleInitialPosition[2]])    

    LeftHipPlot.set_data([WaistInitialPosition[0], LeftHipInitialPosition[0]], [WaistInitialPosition[1], LeftHipInitialPosition[1]])
    LeftHipPlot.set_3d_properties([WaistInitialPosition[2], LeftHipInitialPosition[2]])

    LeftThighPlot.set_data([LeftHipInitialPosition[0], LeftKneeInitialPosition[0]], [LeftHipInitialPosition[1], LeftKneeInitialPosition[1]])
    LeftThighPlot.set_3d_properties([LeftHipInitialPosition[2], LeftKneeInitialPosition[2]])

    LeftLegPlot.set_data([LeftKneeInitialPosition[0], LeftAnkleInitialPosition[0]], [LeftKneeInitialPosition[1], LeftAnkleInitialPosition[1]])
    LeftLegPlot.set_3d_properties([LeftKneeInitialPosition[2], LeftAnkleInitialPosition[2]])

# Setup the kinematic model
KinematicModelSetup()

# Set the initial pose
SetInitialPose()

# Initial epoch
Epoch = 1
StopEpoch = 500

# Listen for incoming datagrams
while(True):

    # Get the data for the next IMU
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]

    NextData = message.decode("ASCII")
    print("ASCII Data: " + NextData)

    # If there is valid data 
    if len(NextData) > 0:

        # If the defined StopEpoch epochs have passed, stop the system
        #if Epoch > StopEpoch:
        #    KneeResults.close()
        #    AnkleResults.close()
        #    exit()

        # Split the next data string
        CurrentQuat = re.split(' ', NextData)

        # Get the IMU number for the current IMU
        IMUNumber = int(CurrentQuat[1])

        # Extract the quaternion data for the current IMU
        Qw = float(CurrentQuat[2])
        Qx = float(CurrentQuat[3])
        Qy = float(CurrentQuat[4])
        Qz = float(CurrentQuat[5])

        # Get the magnitude of the quaternion for the current IMU
        MagQ = math.sqrt(pow(Qw,2) + pow(Qx,2) + pow(Qy,2) + pow(Qz,2))

        try:
            # Get the unit quaternion for the current IMU
            UnitQw = Qw/MagQ
            UnitQx = Qx/MagQ
            UnitQy = Qy/MagQ
            UnitQz = Qz/MagQ
        except:
            UnitQw = 0.1
            UnitQx = 0.1
            UnitQy = 0.1
            UnitQz = 0.1

        # Restart the complete epoch flag
        CompleteEpochFlag = 0

        # Get the new quaternion data for the current IMU
        match IMUNumber:
            case 0:
                TorsoQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1
            case 1:
                RightUpperArmQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1    
            case 2:
                RightForearmQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1  
            case 3:
                LeftUpperArmQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1
            case 4:
                LeftForearmQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1 

            # We are not currently using IMU 5 and 6 (left and right hip)
            case 7:
                RightThighQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1
            case 8:
                LeftThighQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1
            case 9:
                RightLegQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1
            case 10:
                LeftLegQ = Quaternion(UnitQw, UnitQx, UnitQy, UnitQz)
                InitialiseIMUCount += 1

        # If all IMU quaternions have been inititally defined
        if InitialiseIMUCount == IMUsInUse:

            # If all the IMU quaternions have been stored for the current epoch
            CompleteEpochFlag = 1

        # Initial conditions
        if FirstEpochFlag & CompleteEpochFlag:

            # Define the initial quaternions for each body part
            InitialTorsoQ = TorsoQ
            InitialRightUpperArmQ = RightUpperArmQ
            InitialRightForearmQ = RightForearmQ
            InitialLeftUpperArmQ = LeftUpperArmQ
            InitialLeftForearmQ = LeftForearmQ
            
            InitialRightThighQ = RightThighQ
            InitialRightLegQ = RightLegQ
            InitialLeftThighQ = LeftThighQ
            InitialLeftLegQ = LeftLegQ
            
            # Define the initial pose for each body part THIS STEP CAN BE CUT OUT AND INITIAL POSES USED
            
            InitialHeadV = HeadInitialPose
            InitialUpperTorsoV = UpperTorsoInitialPose
            InitialLowerTorsoV = LowerTorsoInitialPose
            InitialRightShoulderV = RightShoulderInitialPose
            InitialRightUpperArmV = RightUpperArmInitialPose
            InitialRightForearmV = RightForearmInitialPose
            InitialLeftShoulderV = LeftShoulderInitialPose
            InitialLeftUpperArmV = LeftUpperArmInitialPose
            InitialLeftForearmV = LeftForearmInitialPose
            
            InitialRightHipV = RightHipInitialPose
            InitialRightThighV = RightThighInitialPose
            InitialRightLegV = RightLegInitialPose
            InitialLeftHipV = LeftHipInitialPose
            InitialLeftThighV = LeftThighInitialPose
            InitialLeftLegV = LeftLegInitialPose
            
            # Clear first epoch flag
            FirstEpochFlag = 0

        # Main update
        if FirstEpochFlag != 1:

            # Update the quaternions
            TorsoUpdatedQ = TorsoQ / InitialTorsoQ
            RightUpperArmUpdatedQ = RightUpperArmQ / InitialRightUpperArmQ
            RightForearmUpdatedQ = RightForearmQ / InitialRightForearmQ
            LeftUpperArmUpdatedQ = LeftUpperArmQ / InitialLeftUpperArmQ
            LeftForearmUpdatedQ = LeftForearmQ / InitialLeftForearmQ

            RightThighUpdatedQ = RightThighQ / InitialRightThighQ
            RightLegUpdatedQ = RightLegQ / InitialRightLegQ
            LeftThighUpdatedQ = LeftThighQ / InitialLeftThighQ
            LeftLegUpdatedQ = LeftLegQ / InitialLeftLegQ
            
            # Update vectors
            HeadV = TorsoUpdatedQ.rotate(InitialHeadV)
            UpperTorsoV = TorsoUpdatedQ.rotate(InitialUpperTorsoV)
            RightShoulderV = TorsoUpdatedQ.rotate(InitialRightShoulderV)
            RightUpperArmV = RightUpperArmUpdatedQ.rotate(InitialRightUpperArmV)
            RightForearmV = RightForearmUpdatedQ.rotate(InitialRightForearmV)
            LeftShoulderV = TorsoUpdatedQ.rotate(InitialLeftShoulderV)
            LeftUpperArmV = LeftUpperArmUpdatedQ.rotate(InitialLeftUpperArmV)
            LeftForearmV = LeftForearmUpdatedQ.rotate(InitialLeftForearmV)

            RightHipV = TorsoUpdatedQ.rotate(InitialRightHipV)
            RightThighV = RightThighUpdatedQ.rotate(InitialRightThighV)
            RightLegV = RightLegUpdatedQ.rotate(InitialRightLegV)
            LeftHipV = TorsoUpdatedQ.rotate(InitialLeftHipV)
            LeftThighV = LeftThighUpdatedQ.rotate(InitialLeftThighV)
            LeftLegV = LeftLegUpdatedQ.rotate(InitialLeftLegV)

            UpperTorsoV = UpperTorsoV
            LowerTorsoV = -1*(UpperTorsoV)

            # Update the kinematic model pose using VECTOR SUMMATION
            HeadCoords = UpperTorsoV

            RightShoulderCoords = UpperTorsoV
            RightArmCoords = UpperTorsoV + RightShoulderV
            RightForearmCoords = UpperTorsoV + RightShoulderV + RightUpperArmV

            LeftShoulderCoords = UpperTorsoV
            LeftArmCoords = UpperTorsoV + LeftShoulderV
            LeftForearmCoords = UpperTorsoV + LeftShoulderV + LeftUpperArmV

            RightHipCoords = LowerTorsoV
            RightThighCoords = LowerTorsoV + RightHipV
            RightLegCoords = LowerTorsoV + RightHipV + RightThighV

            LeftHipCoords = LowerTorsoV
            LeftThighCoords = LowerTorsoV + LeftHipV
            LeftLegCoords = LowerTorsoV + LeftHipV + LeftThighV

            # Upper body -------------------------------------------------------------------------

            # Update head data
            HeadPlot.set_data([HeadCoords[0], HeadCoords[0]+HeadV[0]], [HeadCoords[1], HeadCoords[1]+HeadV[1]])
            HeadPlot.set_3d_properties([HeadCoords[2], HeadCoords[2]+HeadV[2]])
            
            # Update torso data
            UpperTorsoPlot.set_data([Origin[0], UpperTorsoV[0]], [Origin[1], UpperTorsoV[1]])
            UpperTorsoPlot.set_3d_properties([Origin[2], UpperTorsoV[2]])

            LowerTorsoPlot.set_data([Origin[0], LowerTorsoV[0]], [Origin[1], LowerTorsoV[1]])
            LowerTorsoPlot.set_3d_properties([Origin[2], LowerTorsoV[2]])

            # Update right shoulder data
            RightShoulderPlot.set_data([RightShoulderCoords[0], RightShoulderCoords[0]+RightShoulderV[0]], [RightShoulderCoords[1], RightShoulderCoords[1]+RightShoulderV[1]])
            RightShoulderPlot.set_3d_properties([RightShoulderCoords[2], RightShoulderCoords[2]+RightShoulderV[2]])
            
            # Update right arm data
            RightArmPlot.set_data([RightArmCoords[0], RightArmCoords[0]+RightUpperArmV[0]], [RightArmCoords[1], RightArmCoords[1]+RightUpperArmV[1]])
            RightArmPlot.set_3d_properties([RightArmCoords[2], RightArmCoords[2]+RightUpperArmV[2]])

            # Update right forearm data
            RightForearmPlot.set_data([RightForearmCoords[0], RightForearmCoords[0]+RightForearmV[0]], [RightForearmCoords[1], RightForearmCoords[1]+RightForearmV[1]])
            RightForearmPlot.set_3d_properties([RightForearmCoords[2], RightForearmCoords[2]+RightForearmV[2]])

            # Update left shoulder data
            LeftShoulderPlot.set_data([LeftShoulderCoords[0], LeftShoulderCoords[0]+LeftShoulderV[0]], [LeftShoulderCoords[1], LeftShoulderCoords[1]+LeftShoulderV[1]])
            LeftShoulderPlot.set_3d_properties([LeftShoulderCoords[2], LeftShoulderCoords[2]+LeftShoulderV[2]])

            # Update left arm data
            LeftArmPlot.set_data([LeftArmCoords[0], LeftArmCoords[0]+LeftUpperArmV[0]], [LeftArmCoords[1], LeftArmCoords[1]+LeftUpperArmV[1]])
            LeftArmPlot.set_3d_properties([LeftArmCoords[2], LeftArmCoords[2]+LeftUpperArmV[2]])

            # Update left forearm data
            LeftForearmPlot.set_data([LeftForearmCoords[0], LeftForearmCoords[0]+LeftForearmV[0]], [LeftForearmCoords[1], LeftForearmCoords[1]+LeftForearmV[1]])
            LeftForearmPlot.set_3d_properties([LeftForearmCoords[2], LeftForearmCoords[2]+LeftForearmV[2]])
            
            # Lower body -------------------------------------------------------------------------

            # Update right hip data
            RightHipPlot.set_data([RightHipCoords[0], RightHipCoords[0]+RightHipV[0]], [RightHipCoords[1], RightHipCoords[1]+RightHipV[1]])
            RightHipPlot.set_3d_properties([RightHipCoords[2], RightHipCoords[2]+RightHipV[2]])

            # Update right thigh data
            RightThighPlot.set_data([RightThighCoords[0], RightThighCoords[0]+RightThighV[0]], [RightThighCoords[1], RightThighCoords[1]+RightThighV[1]])
            RightThighPlot.set_3d_properties([RightThighCoords[2], RightThighCoords[2]+RightThighV[2]])

            # Update right leg data
            RightLegPlot.set_data([RightLegCoords[0], RightLegCoords[0]+RightLegV[0]], [RightLegCoords[1], RightLegCoords[1]+RightLegV[1]])
            RightLegPlot.set_3d_properties([RightLegCoords[2], RightLegCoords[2]+RightLegV[2]])

            # Update left hip data
            LeftHipPlot.set_data([LeftHipCoords[0], LeftHipCoords[0]+LeftHipV[0]], [LeftHipCoords[1], LeftHipCoords[1]+LeftHipV[1]])
            LeftHipPlot.set_3d_properties([LeftHipCoords[2], LeftHipCoords[2]+LeftHipV[2]])

            # Update left thigh data
            LeftThighPlot.set_data([LeftThighCoords[0], LeftThighCoords[0]+LeftThighV[0]], [LeftThighCoords[1], LeftThighCoords[1]+LeftThighV[1]])
            LeftThighPlot.set_3d_properties([LeftThighCoords[2], LeftThighCoords[2]+LeftThighV[2]])

            # Update left leg data
            LeftLegPlot.set_data([LeftLegCoords[0], LeftLegCoords[0]+LeftLegV[0]], [LeftLegCoords[1], LeftLegCoords[1]+LeftLegV[1]])
            LeftLegPlot.set_3d_properties([LeftLegCoords[2], LeftLegCoords[2]+LeftLegV[2]])

            # Draw the kinematic model 
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

            # Update IMU index
            Index = Index + 1

            # Clear complete epoch flag
            CompleteEpochFlag = 0

            # Send data to the results file for error tracking
            #KneeResults.write(str(RightLegCoords[0]) + " " + str(RightLegCoords[1]) + " " + str(RightLegCoords[2]) + "\n")
            #AnkleResults.write(str(RightLegCoords[0]+RightLegV[0]) + " " + str(RightLegCoords[1]+RightLegV[1]) + " " + str(RightLegCoords[2]+RightLegV[2]) + "\n")

        Epoch = Epoch + 1

    else:
        print("No valid data!")

    


