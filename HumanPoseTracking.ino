//
// IMU-Based 3D Human Pose Tracking System
//
// Author: Aidan Stapleton
//
// Date: 12/10/2023
//

// Includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h" 

// Define initial variables
char ssid[] = "Optus_54B772_EXT";         // Network SSID (name)
char pass[] = "deedygamey6fHGH";          // Network password (use for WPA, or use as key for WEP)
//char NetworkIP[] = "192.168.0.127";     // Laptop IP address
char NetworkIP[] = "192.168.0.32";        // Desktop IP address
unsigned int localPort = 2390;            // local port to listen on
int status = WL_IDLE_STATUS;
const int Precision = 6;
const int IMUsInUse = 9;
const bool EnableCalibration = 0;

// Define the UDP instance
WiFiUDP Udp;

// IMU Identifiers
Adafruit_BNO055 bno0 = Adafruit_BNO055(0);            // Torso    
Adafruit_BNO055 bno1 = Adafruit_BNO055(1);            // Right Upper Arm
Adafruit_BNO055 bno2 = Adafruit_BNO055(2);            // Right Forearm
Adafruit_BNO055 bno3 = Adafruit_BNO055(3);            // Left Upper Arm
Adafruit_BNO055 bno4 = Adafruit_BNO055(4);            // Left Forearm

Adafruit_BNO055 bno5 = Adafruit_BNO055(5);            // Not being used
Adafruit_BNO055 bno6 = Adafruit_BNO055(6);            // Not being used
Adafruit_BNO055 bno7 = Adafruit_BNO055(7);            // Right Thigh
Adafruit_BNO055 bno8 = Adafruit_BNO055(8);            // Left Thigh
Adafruit_BNO055 bno9 = Adafruit_BNO055(9);            // Right Leg
Adafruit_BNO055 bno10 = Adafruit_BNO055(10);          // Left Leg    

// IMU Connection Status
bool IMUConnectionStatus[11] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

#define Multiplexer1 0x70
#define Multiplexer2 0x71

// This loop runs once
void setup() {

  // Set the pins for wifi connection
  WiFi.setPins(8,7,4,2);

  // Begin I2C communication with the multiplexer(s)
  Wire.begin();

  // Check for the presence of the shield (ATWINC1500)
  if (WiFi.status() == WL_NO_SHIELD){

    Serial.println("WiFi shield not present");

    // Don't continue
    while (true);
  }

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {

    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // Wifi status
  Serial.println("Connected to wifi");
  PrintWiFiStatus();

  // Connect to the UDP socket and flush
  Serial.println("\nStarting UDP connection...");
  Udp.begin(localPort);
  Udp.flush();

  Serial.println("Orientation Sensor Test");
  Serial.println(" ");

  // Setup each IMU
  SetupIMU();

  if (EnableCalibration){
    
    // Calibrate each IMU
    CalibrateIMU();
  }
  
  // Detect I2C addresses
  I2CScanner();

  // Tell the user to get into pose
  Serial.println("Get into the initial pose!");
}

// Function to interface with the TCA9548A multiplexers
void TCA9548A(uint8_t bus){

  // IMU bus for multiplexer 1 & 2
  uint8_t bus1;
  uint8_t bus2;

  // Configure multiplexer buses
  if(bus <= 4){
    bus1 = 1 << bus;
    bus2 = 0;
  }
  else{
    bus1 = 0;
    bus2 = 1 << (bus - 5);
  }

  // Data from multiplexer 1
  Wire.beginTransmission(Multiplexer1);
  Wire.write(bus1);
  Wire.endTransmission();

  // Data from multiplexer 2
  Wire.beginTransmission(Multiplexer2);
  Wire.write(bus2);
  Wire.endTransmission();

  delay(1);
}

// Function to initialise the BNO055 IMUs
void SetupIMU() {

  // IMU 0
  TCA9548A(0); 
  if(!bno0.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 0 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[0] = 0;
  }

  // IMU 1
  TCA9548A(1);
  //bno1.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
  if(!bno1.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 1 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[1] = 0;
  }

  // IMU 2
  TCA9548A(2);
  if(!bno2.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 2 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[2] = 0;
  }

  // IMU 3
  TCA9548A(3); 
  if(!bno3.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 3 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[3] = 0;
  }

  // IMU 4
  TCA9548A(4); 
  if(!bno4.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 4 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[4] = 0;
  }

  // IMU 5
  TCA9548A(5); 
  if(!bno5.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 5 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[5] = 0;
  }

  // IMU 6
  TCA9548A(6); 
  if(!bno6.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 6 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[6] = 0;
  }

  // IMU 7
  TCA9548A(7); 
  if(!bno7.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 7 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[7] = 0;
  }

  // IMU 8
  TCA9548A(8); 
  if(!bno8.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 8 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[8] = 0;
  }

  // IMU 9
  TCA9548A(9); 
  if(!bno9.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 9 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[9] = 0;
  }

  // IMU 10
  TCA9548A(10); 
  if(!bno10.begin()){

    // There was a problem detecting the BNO055
    Serial.println("BNO055 IMU 10 was not detected, check your wiring or I2C ADDR!");

    // Set the status to OFF
    IMUConnectionStatus[10] = 0;
  }

  // Use an external clock for each IMU
  bno0.setExtCrystalUse(true);
  bno1.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);
  bno3.setExtCrystalUse(true);
  bno4.setExtCrystalUse(true);
  bno5.setExtCrystalUse(true);
  bno6.setExtCrystalUse(true);
  bno7.setExtCrystalUse(true);
  bno8.setExtCrystalUse(true);
  bno9.setExtCrystalUse(true);
  bno10.setExtCrystalUse(true);
}

// This loop runs continuously
void loop(void) {

  sensors_event_t event;
  imu::Quaternion NextQuat;
  int NextIMU;

  // Loop through each IMU
  for (uint8_t IMUIndex = 0; IMUIndex <= (IMUsInUse-1); IMUIndex++){                  

    // If the IMU number is greater than 4, then increment by 2
    if(IMUIndex > 4){
      NextIMU = IMUIndex + 2;
    }
    else{
      NextIMU = IMUIndex;
    }

    // Select the next IMU
    TCA9548A(NextIMU);   

    if(NextIMU == 0){
      bno0.getEvent(&event);
      NextQuat = bno0.getQuat();
    }
    else if(NextIMU == 1){
      bno1.getEvent(&event);
      NextQuat = bno1.getQuat();
    }
    else if(NextIMU == 2){
      bno2.getEvent(&event);
      NextQuat = bno2.getQuat();
    }
    else if(NextIMU == 3){
      bno3.getEvent(&event);
      NextQuat = bno3.getQuat();
    }
    else if(NextIMU == 4){
      bno4.getEvent(&event);
      NextQuat = bno4.getQuat();
    }

    // NOTE: We are not currently using IMU 5 and 6 (left and right hip)

    else if(NextIMU == 7){
      bno7.getEvent(&event);
      NextQuat = bno7.getQuat();
    }
    else if(NextIMU == 8){
      bno8.getEvent(&event);
      NextQuat = bno8.getQuat();
    }
    else if(NextIMU == 9){
      bno9.getEvent(&event);
      NextQuat = bno9.getQuat();
    }
    else if(NextIMU == 10){
      bno10.getEvent(&event);
      NextQuat = bno10.getQuat();
    }

    // Define Quaternion components
    float wQuat = NextQuat.w();
    float xQuat = NextQuat.x();
    float yQuat = NextQuat.y();
    float zQuat = NextQuat.z();

    // Get each quaternion value as a string with 
    // the predefined precision
    String IMU_Index = String(NextIMU);
    String wQuatStr = String(wQuat, Precision);
    String xQuatStr = String(xQuat, Precision);
    String yQuatStr = String(yQuat, Precision);
    String zQuatStr = String(zQuat, Precision);

    // Combine the next quaternion as a string with the IMU index number 
    String NextQuaternion = "IMU " + IMU_Index + " " + wQuatStr + " " + xQuatStr + " " + yQuatStr + " " + zQuatStr;
    Serial.println(NextQuaternion);

    // Send the next quaternion as a string inside a UDP data packet
    Udp.beginPacket(NetworkIP, localPort);
    Udp.print(NextQuaternion);
    Udp.endPacket();

    // Wait for all outgoing characters to be sent and clear the output buffer
    Udp.flush();

    // Wait 100 milliseconds before getting new data (10Hz) NOTE: BNO055s run at 100Hz
    delay(100);                                 
  }
}

// Function to scan the I2C bus for connected devices
void I2CScanner(){

  byte Error, Address;
  int NumDevices;

  Serial.println("Scanning for I2C Peripheral Devices...");

  NumDevices = 0;
  for(Address = 1; Address < 127; Address++){

    // Uses the return value of the function Write.endTransmisstion to check
    // for devices.
    Wire.beginTransmission(Address);
    Error = Wire.endTransmission();

    if (Error == 0){

      Serial.print("I2C device found at address 0x");
      if (Address < 16) 
        Serial.print("0");
      Serial.print(Address, HEX);
      Serial.println("  !");

      NumDevices++;
    }
    else if (Error==4){

      Serial.print("Unknown error at address 0x");
      if (Address < 16) 
        Serial.print("0");
      Serial.println(Address, HEX);
    }    
  }

  // If there were no I2C devices detected
  if (NumDevices == 0){

    Serial.println("No I2C devices found\n");
  }
  else{

    Serial.println("done\n");
  }

  // Wait for 5 seconds for the next I2C scan
  delay(5000);           
}

// Function to display sensor calibration status
void CalibrateIMU(void){

  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */

  Serial.println("Beginning IMU Calibration");

  // Define IMU calibration status variables
  uint8_t system, gyro, accel, mag;

  for(int IMU = 1; IMU <= (IMUsInUse - 1); IMU++){   

    // Reset calibration status for the next IMU
    system = gyro = accel = mag = 0;

    // The data should be ignored until the system calibration is > 0
    Serial.print("Calibrate IMU ");
    Serial.println(IMU);
    TCA9548A(IMU); 

    if (IMUConnectionStatus[IMU] == 0){
      Serial.print("IMU ");
      Serial.print(IMU);
      Serial.println(" is not connected, calibration not required.");
    }
    else{

      switch (IMU){

      // IMU 0
      case 0:
        
        // Get the current calibration
        bno0.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno0.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 0 Calibrated!");
        break;

      // IMU 1
      case 1:
        
        // Get the current calibration
        bno1.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno1.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 1 Calibrated!");
        break;

      // IMU 2
      case 2:

        // Get the current calibration
        bno2.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno2.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 2 Calibrated!");
        break;

      // IMU 3
      case 3:

        // Get the current calibration
        bno3.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno3.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 3 Calibrated!");
        break;

      // IMU 4
      case 4:

        // Get the current calibration
        bno4.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno4.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 4 Calibrated!");
        break;

      // IMU 5
      case 5:

        // Get the current calibration
        bno5.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno5.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 5 Calibrated!");
        break;

      // IMU 6
      case 6:

        // Get the current calibration
        bno6.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno6.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 6 Calibrated!");
        break;

      // IMU 7
      case 7:

        // Get the current calibration
        bno7.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno7.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 7 Calibrated!");
        break;

      // IMU 8
      case 8:

        // Get the current calibration
        bno8.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno8.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 8 Calibrated!");
        break;

      // IMU 9
      case 9:

          // Get the current calibration
        bno9.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno9.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 9 Calibrated!");
        break;

      // IMU 10
      case 10:

        // Get the current calibration
        bno10.getCalibration(&system, &gyro, &accel, &mag);

        // Loop until fully calibrated
        while ((system < 3) || (accel < 3) || (gyro < 3) || (mag < 3)){

          // Get the current calibration
          bno10.getCalibration(&system, &gyro, &accel, &mag);

          // Display the current calibration status for each sensor
          Serial.print("Sys:");
          Serial.print(system, DEC);
          Serial.print(" G:");
          Serial.print(gyro, DEC);
          Serial.print(" A:");
          Serial.print(accel, DEC);
          Serial.print(" M:");
          Serial.println(mag, DEC);
        }

        // Successfully calibrated
        Serial.print("IMU 10 Calibrated!");
        break;

      // Default case
      default:
        Serial.print("Invalid IMU Number!");
        break;
      }
  }
  }
}

void PrintWiFiStatus(){

  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("Local IP Address: ");
  Serial.println(ip);

  Serial.print("Remote IP Address: ");
  Serial.println(Udp.remoteIP());

  // Print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


