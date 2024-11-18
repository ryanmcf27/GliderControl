/*
  Teensy_Descent

  Custom code for Iowa State University's 2024-2025 Design Build Fly (DBF) team's X-1 glider
  
*/

/* Libraries */
#include <TaskScheduler.h> //for more info refer to source repository: https://github.com/arkhipenko/TaskScheduler/tree/master
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "src/MPU6050/MPU6050.h"

/* Method Prototypes */
void updateGPS();
void updateIMU();
void strobe();
void calculate_IMU_error();

// set up a global struct to store our sensor data
struct SensorDataStruct {
  float gpsLat = 0.0;     //X-1's latitude (degrees)
  float gpsLng = 0.0;     //X-1's longitude (degrees)
  float roll = 0.0;       //roll angle (degrees) - may need to add offset if IMU isn't mounted flat
  float pitch = 0.0;      //pitch angle (degrees) - may need to add offset if IMU isn't mounted flat
  float yaw = 0.0;        //yaw angle (degrees) - may need to add offset if IMU isn't mounted flat
  float accelX = 0.0;     //X-1's acceleration in the X direction
  float accelY = 0.0;     //X-1's acceleration in the Y direction
  float accelZ = 0.0;     //X-1's acceleration in the Z direction
};

// global variables to store the coordinates of our target location
float targetLat = 32.265617;
float targetLng = -111.273524;

// set up enum for the glider's flight state, initialize to PRERELEASE state
enum {PRERELEASE, DESCENT, ERROR_STATE} flightState = PRERELEASE;

/* Constants and Pins */
static const int RXPin = 0, TXPin = 1;
static const uint32_t MONITOR_BAUD = 115200, GPS_BAUD = 9600;
#define ACCEL_SCALE_FACTOR 1.0  //full scale accelerometer range (16384.0 for 2 G's) - reference dRehmFlight, using 1 for now to see raw IMU output
#define GYRO_SCALE_FACTOR 1.0     //full scale accelerometer range (131.0 for 250 degrees/sec) - reference dRehmFlight, using 1 for now to see raw IMU output
//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

/* Tasks */
#define GPS_TASK_RATE 250
#define IMU_TASK_RATE 250
#define STROBE_TASK_RATE 250
Task gpsTask(GPS_TASK_RATE, TASK_FOREVER, &updateGPS);      //update GPS global variables every GPS_TASK_RATE ms, will be enabled from the start
Task imuTask(IMU_TASK_RATE, TASK_FOREVER, &updateIMU);      //update IMU global variables every IMU_TASK_RATE ms, will be enabled from the start
Task strobeTask(STROBE_TASK_RATE, TASK_FOREVER, &strobe);   //strobe lights every STROBE_TASK_RATE ms, won't be enabled until release

/* Declaring Components */
Scheduler runner;                     //declare our task scheduler
MPU6050 mpu6050;                      //declare MPU6050 object for our IMU
TinyGPSPlus gps;                      //declare TinyGPSPlus object to handle GPS data (convert raw data to be more user friendly)
SoftwareSerial ss_GPS(RXPin, TXPin);  //declare serial connection to the GPS module
SensorDataStruct sensorData;          //declare struct to hold our sensor data

void setup() {
  Serial.begin(MONITOR_BAUD);
  ss_GPS.begin(GPS_BAUD);
  runner.init();
  Serial.println("Initialized scheduler");
  
  runner.addTask(gpsTask);
  Serial.println("added gpsTask");
  
  runner.addTask(imuTask);
  Serial.println("added imuTask");

  runner.addTask(strobeTask);
  Serial.println("added strobeTask");

  delay(1000);
  
  gpsTask.enable();
  Serial.println("Enabled gpsTask");
  imuTask.enable();
  Serial.println("Enabled imuTask");

  delay(1000);

  calculate_IMU_error();
  delay(1000);
}

void loop() {
  runner.execute();
  switch(flightState){
    case PRERELEASE:
      //checks to see if X-1 has been released based on limit switch
      //if(---){
        //update flightState
       // flightState = DESCENT;
      //}
      break;
    case DESCENT:
      //TODO - logic for steering descent into target zone
      break;
    case ERROR_STATE:
      Serial.println("In ERROR flight state!!!");
      break;
  }
}

void updateGPS() {
  // get GPS data from i2C
  if (ss_GPS.available()){
    gps.encode(ss_GPS.read());
    if(gps.location.isUpdated()){
      //if the received gps data is successfully encoded, update global variables
      sensorData.gpsLat = gps.location.lat();
      sensorData.gpsLng = gps.location.lng();
      
      //for testing
      char buffer [50];
      sprintf(buffer, "Latitude: %f, Longitude: %f", sensorData.gpsLat, sensorData.gpsLng);
      Serial.println(buffer);
    }
  }
}

void updateIMU() {
  // Get new raw IMU values
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Update IMU global variables with corrected and scaled values
  sensorData.accelX = (AcX / ACCEL_SCALE_FACTOR) - AccErrorX;
  sensorData.accelY = (AcY / ACCEL_SCALE_FACTOR) - AccErrorY;
  sensorData.accelZ = (AcZ / ACCEL_SCALE_FACTOR) - AccErrorZ;
  sensorData.roll = (GyX / GYRO_SCALE_FACTOR) - GyroErrorX;
  sensorData.pitch = (GyY / GYRO_SCALE_FACTOR) - GyroErrorY;
  sensorData.yaw = (GyZ / GYRO_SCALE_FACTOR) - GyroErrorZ;
}

void strobe() {
  // TODO - method to strobe LEDs
}


void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  //Credit: dRehmFlight - https://github.com/nickrehm/dRehmFlight/blob/master/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;  //temp vars for getting raw values from the IMU
  float AccX, AccY, AccZ, GyroX, GyroY, GyroZ; //temp vars for scaling IMU values
  
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}